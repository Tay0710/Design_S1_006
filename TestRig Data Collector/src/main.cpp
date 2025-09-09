// 


// lib's for each of the sensors?
// Components:
// IMU, OF, SD Card, 4*ToF, 4*Ultrasonic

#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <ICM45686.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <Bitcraze_PMW3901.h>

// Optical flow SPI pins (pins for Owen's ESP32)
#define MOSI1 11 // OF, VSPI
#define CLK1 12
#define MISO1 13
#define CS1 10

#define AP_SDI2 37 // IMU / ICM - PSRAM
#define AP_CLK2 36 // SDO = MISO; SDI = MOSI
#define AP_SDO2 38
#define AP_CS2 35

#define SDCLK 47 // SD Card, HSPI
#define SDMISO 48
#define SDMOSI 21
#define SDCS 39

#define SDA1 8   // 
#define SCL1 9   // 

#define SDA2 6
#define SCL2 7
// ToF inputs. 
#define PENA 41 // Power enable
#define RESET 40
#define LPN 42

#define US1 14
#define US2 15
#define US3 16
#define US4 4
#define US5 5

#define BOOT_PIN 0
bool mode = false;         // toggled by BOOT button
unsigned long lastPress = 0;
const unsigned long debounceDelay = 200; // ms


// VL53L7CX new I2C addresses for 2nd Sensor on I2C bus
#define CHANGE_ADDR 0x30

// Declaring the two I2C buses
TwoWire I2C1 = TwoWire(0); // use hardware I2C port 0
TwoWire I2C2 = TwoWire(1); // use hardware I2C port 1

// Sensor objects and measurement data
SparkFun_VL53L5CX sensorS1;
VL53L5CX_ResultsData measurementDataS1; // Result data class structure, 1356 byes of RAM
SparkFun_VL53L5CX sensorS2;
VL53L5CX_ResultsData measurementDataS2;
SparkFun_VL53L5CX sensorR;
VL53L5CX_ResultsData measurementDataR;
SparkFun_VL53L5CX sensorF;
VL53L5CX_ResultsData measurementDataF;

// Chip select assignment
// Create SPI buses
SPIClass vspi(VSPI);   // Optical Flow
SPIClass hspi(HSPI);   // SD Card
SPIClass psramSPI(FSPI); // Custom: using Flash/PSRAM bus for IMU
Bitcraze_PMW3901 flow(CS1);
ICM456xx IMU(psramSPI, AP_CS2);

// Global file handles
File imuFile;
File ofFile;
File tofFile;
// For if we do separate ToF files
// File ToFS1; // side 1
// File ToFS2; // side 2
// File ToFR;  // roof
// File ToFF;  // floor

// File Names
const char* imuFileName = "/imu_ICM45686.csv";
const char* tofFileName = "/4x_tof_L7.csv";
const char* ofFileName = "/of_PMW3901.csv";

// Variables for PMW3901
char frame[35*35]; //array to hold the framebuffer
int16_t deltaX,deltaY;

// Size of ToF image array
int S1imageResolution = 0; // Same for S2
int RimageResolution  = 0; // Same for F

///////////////////////////////////////////////////////
// CHECK THAT THESE CHAR SIZES are LARGE ENOUGH !!!!
//////////////////////////////////////////////////////
char imuBuf[128];
char ofBuf[64];

// ToF data can be recorded all at the same time?
char tofBuf[1024]; // All 4 ToFs in one buffer
// char tofS1Buf[384]; // 8x8 resolution, with 4 character + 1 space ~ 384 chars
// char tofS2Buf[384]; 
// char tofRBuf[96]; // 4x4 resolution ~ 96 chars (16*5 = 80)
// char tofFBuf[96];

// IMU - Calibration offsets
float calibAccelX = 0, calibAccelY = 0, calibAccelZ = 0;
float calibGyroX  = 0, calibGyroY  = 0, calibGyroZ  = 0;

float  G_rating = 4;      // 2/4/8/16/32 g
float  dps_rating = 1000; // 15.625/31.25/62.5/125/250/500/1000/2000/4000 dps

// Timing control
unsigned long lastIMUtime = 0;
unsigned long lastOFtime = 0;
unsigned long lastTOFtime = 0;
const unsigned long imuInterval = 625;    // microseconds → ~1600 Hz
const unsigned long ofInterval = 10000;   // microseconds → ~100 Hz
const unsigned long tofInterval = 250000;   // microseconds → ~4 Hz // Side Tof should be every 0.25s and Roof/ Floor ToF should be every 0.5s. 
// const unsigned long S1tofInterval = 250000;
// const unsigned long S2tofInterval = 250000;
// const unsigned long RtofInterval  = 250000; 
// const unsigned long FtofInterval  = 250000;

// ---- Calibration function ----
void calibrateIMU(int samples) {
  long sumAx = 0, sumAy = 0, sumAz = 0;
  long sumGx = 0, sumGy = 0, sumGz = 0;

  Serial.println("Starting IMU calibration...");

  unsigned long startTime = millis();

  for (int i = 0; i < samples; i++) {
    inv_imu_sensor_data_t imu_data;
    IMU.getDataFromRegisters(imu_data);

    sumAx += imu_data.accel_data[0];
    sumAy += imu_data.accel_data[1];
    sumAz += imu_data.accel_data[2];
    sumGx += imu_data.gyro_data[0];
    sumGy += imu_data.gyro_data[1];
    sumGz += imu_data.gyro_data[2];

    // delay(2); // ~500 Hz - Delay is not required (ignore CHATGPT suggestion)
  }

  unsigned long endTime = millis();
  unsigned long duration = endTime - startTime;

  float avgAx = (float)sumAx / samples;
  float avgAy = (float)sumAy / samples;
  float avgAz = (float)sumAz / samples;
  float avgGx = (float)sumGx / samples;
  float avgGy = (float)sumGy / samples;
  float avgGz = (float)sumGz / samples;

  calibAccelX = avgAx * G_rating / 32768.0;
  calibAccelY = avgAy * G_rating / 32768.0;
  calibAccelZ = avgAz * G_rating / 32768.0 - 1;
  calibGyroX  = avgGx * dps_rating / 32768.0;
  calibGyroY  = avgGy * dps_rating / 32768.0;
  calibGyroZ  = avgGz * dps_rating / 32768.0;

  Serial.println("Calibration complete:");
  Serial.printf("Accel offsets: %.6f, %.6f, %.6f\n", calibAccelX, calibAccelY, calibAccelZ);
  Serial.printf("Gyro  offsets: %.6f, %.6f, %.6f\n", calibGyroX, calibGyroY, calibGyroZ);
  Serial.printf("Calibration took %lu ms (%lu samples at ~%lu Hz)\n",
                duration, samples, (samples * 1000UL) / duration);
}


void setup()
{
  
  Serial.begin(115200);
  delay(1000);
  pinMode(BOOT_PIN, INPUT_PULLUP);
  delay(10);


  // SPI Setup
  vspi.begin(CLK1, MISO1, MOSI1, CS1);
  hspi.begin(SDCLK, SDMISO, SDMOSI, SDCS);
  psramSPI.begin(AP_CLK2, AP_SDO2, AP_SDI2, AP_CS2);

  // ICM45686 Begin - using psramSPI.
  if (IMU.begin() != 0) {
    Serial.println("ICM456xx initialization failed");
    while (1);
  }
  // Configure accelerometer and gyro
  IMU.startAccel(1600, G_rating);     // Max 6400Hz; 100 Hz, ±2/4/8/16/32 g
  IMU.startGyro(1600, dps_rating);    // 100 Hz, ±15.625/31.25/62.5/125/250/500/1000/
  delay(100); // Delay needed to ensure proper calibration
  Serial.println("Do not move drone while calibrating the ICM.");
  calibrateIMU(1000);
  // PMW3901 begin - using vspi
  if (!flow.begin()) {
      Serial.println("PMW3901 initialization failed. Check wiring!");
      while (1);  // stop if not found
  }
  delay(100);
  flow.enableFrameBuffer(); 
  // Initialize SD card - using hspi
  if (!SD.begin(SDCS, hspi)) {
    Serial.println("SD init failed!");
    while (1);
  }

  /*
4 ToF sensor code. 
*/
  I2C1.begin(); // This resets I2C bus to 100kHz
  I2C1.setClock(1000000); //Sensor (L7) has max I2C freq of 1MHz
  I2C2.begin(); 
  I2C2.setClock(1000000); 
  Serial.println("Clock Has been Set for I2C's!");

// Address Reset sequence for ToFL7 sensors.
// Activating PWR_EN (Make High). 
  pinMode(PENA, OUTPUT);
  digitalWrite(PENA, HIGH); 
  delay(100);   
// Activating I2C Reset pin to reset the addresses (Pulse High). 
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH); 
  delay(100); 
  digitalWrite(RESET, LOW); 
  delay(100); 
// Deactivating PWR_EN (Make Low). Reseting Sensors.  
  digitalWrite(PENA, LOW); 
  delay(100);   
  digitalWrite(PENA, HIGH); // Make High again.
  delay(100);  
  // Configure LPn pins
  pinMode(LPN, OUTPUT);
  // Set Sensor 1 LPn low (Deactivate I2C communication). 
  digitalWrite(LPN, LOW); // One LPn should be set HIGH permanently
  delay(100); 

  // I2C Bus split: S1 + R on I2C1; S2 + F on I2C2
  // Have to change the address of the ToFs with no LPN pin attached.
  // Make R & F the changed Address sensors. 


  // Change Sensor Address to 0x30 after calling begin()
    if (!sensorR.begin()) { 
    Serial.println("Sensor R not found at 0x29!");
    while (1);
  }
    if (!sensorF.begin()) { 
    Serial.println("Sensor F not found at 0x29!");
    while (1);
  }
  sensorR.setAddress(CHANGE_ADDR);
  sensorF.setAddress(CHANGE_ADDR);
  sensorR.setResolution(4 * 4);
  sensorF.setResolution(4 * 4);
  // Using 4x4, min frequency is 1Hz and max is 60Hz
  // Using 8x8, min frequency is 1Hz and max is 15Hz
  RimageResolution = sensorR.getResolution();
  Serial.println("Sensor's initialized successfully at 0x30");
  delay(50);

  // Default Address Sensor initialization - Set Sensor LPn HIGH (Activate I2C communication). 
  digitalWrite(LPN, HIGH); // Other LPn should still be set HIGH
  delay(100); 

  if (!sensorS1.begin()) {
    Serial.println("Sensor S1 not found at 0x29!");
    while (1);
  } 
  if (!sensorS2.begin()) {
    Serial.println("Sensor S2 not found at 0x29!");
    while (1);
  }
  sensorS1.setResolution(8 * 8);
  sensorS1.setResolution(8 * 8);
  S1imageResolution = sensorS1.getResolution();
  Serial.println("Sensor 1 initialized successfully at 0x29");
  delay(50);

  // // Set Frequency
  // sensorR.setRangingFrequency(15);
  // sensorF.setRangingFrequency(15);
  // sensorS1.setRangingFrequency(15);
  // sensorS2.setRangingFrequency(15);
  // Start ranging on both sensors. 
  Serial.println("Starting ranging of ToFL7  sensors...");
  sensorR.startRanging();
  sensorF.startRanging();
  sensorS1.startRanging();
  sensorS2.startRanging();
  Serial.println("Sensors are now ranging.");


  // Init IMU CSV
  SD.remove(imuFileName);
  File imu = SD.open(imuFileName, FILE_WRITE);
  imu.println("time,gyro x,gyro y,gyro z,accel x,accel y,accel z");
  imu.close();
  // Init ToF CSV
  SD.remove(tofFileName);
  File tof = SD.open(tofFileName, FILE_WRITE);
  tof.print("time");
  for(int i=0; i<(2*(RimageResolution+S1imageResolution)); i++) tof.print(",D"+String(i));
  tof.println();
  tof.close();
  // Init OF CSV
  SD.remove(ofFileName);
  File of = SD.open(ofFileName, FILE_WRITE);
  of.print("time");
  of.print(",deltaX,deltaY");
  of.println();
  of.close();






  Serial.println("Entering Loop!");
}

// Helper: append unsigned long to buffer, returns number of chars
int appendULong(char* buf, unsigned long val) {
    char temp[11]; // max 10 digits + null
    int i = 0;
    if(val == 0) {
        buf[0] = '0';
        return 1;
    }
    while(val > 0) {
        temp[i++] = '0' + (val % 10);
        val /= 10;
    }
    for(int j = 0; j < i; j++) buf[j] = temp[i - j - 1];
    return i;
}

// Write timestamp in seconds.microseconds
int appendTimestamp(char* buf, unsigned long micros_val) {
    unsigned long seconds = micros_val / 1000000;
    unsigned long us      = micros_val % 1000000;
    int idx = 0;
    idx += appendULong(buf + idx, seconds);
    buf[idx++] = '.';
    // Pad microseconds with leading zeros
    int digits = 6;
    char temp[6];
    for(int i = 5; i >= 0; i--) {
        temp[i] = '0' + (us % 10);
        us /= 10;
    }
    for(int i = 0; i < 6; i++) buf[idx++] = temp[i];
    return idx;
}

void logIMU() {
  unsigned long now = micros();
  if (now - lastIMUtime < imuInterval) return;  
  lastIMUtime = now;
  // Serial.print("entering_imu:");
  // Serial.printf("%.9f",now/1000000.0);
  if (!imuFile) return; // if file is not open; skip!

  inv_imu_sensor_data_t imu_data;
  IMU.getDataFromRegisters(imu_data);

  float gx = imu_data.gyro_data[0]*dps_rating/32768.0 - calibGyroX;
  float gy = imu_data.gyro_data[1]*dps_rating/32768.0 - calibGyroY;
  float gz = imu_data.gyro_data[2]*dps_rating/32768.0 - calibGyroZ;
  float ax = imu_data.accel_data[0]*G_rating/32768.0 - calibAccelX;
  float ay = imu_data.accel_data[1]*G_rating/32768.0 - calibAccelY;
  float az = imu_data.accel_data[2]*G_rating/32768.0 - calibAccelZ;

  int idx = appendTimestamp(imuBuf, now);
  idx += snprintf(imuBuf + idx, sizeof(imuBuf) - idx, ",%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", gx, gy, gz, ax, ay, az);
  imuFile.write((uint8_t*)imuBuf, idx);
}

// Fast integer to string conversion, returns number of chars written
int intToStr(int val, char* buf) {
  char temp[6]; // max 3500 -> 4 digits + null
  int i = 0;
  if(val == 0) {
      buf[0] = '0';
      return 1;
  }
  while(val > 0) {
      temp[i++] = '0' + (val % 10);
      val /= 10;
  }
  // reverse
  for(int j = 0; j < i; j++) {
      buf[j] = temp[i - j - 1];
  }
  return i;
}

void logToF() {
  unsigned long now = micros();
  if (now - lastTOFtime < tofInterval) return;  
  lastTOFtime = now;
  if (!tofFile) return;

  int idx = appendTimestamp(tofBuf, now); // write timestamp
  if(sensorS1.isDataReady() && sensorS1.getRangingData(&measurementDataS1)) {
      for(int i = 0; i < S1imageResolution; i++) { 
          tofBuf[idx++] = ',';      
          const uint8_t stat  = measurementDataS1.target_status[i];
          if (stat == 5){         
          idx += intToStr(measurementDataS1.distance_mm[i], tofBuf + idx); // fast int -> string
          }
          else{
            tofBuf[idx++] = 'X';
          }
      }
  }
  if(sensorS2.isDataReady() && sensorS2.getRangingData(&measurementDataS2)) {
      for(int i = 0; i < S1imageResolution; i++) { 
          tofBuf[idx++] = ',';      
          const uint8_t stat  = measurementDataS2.target_status[i];
          if (stat == 5){         
          idx += intToStr(measurementDataS2.distance_mm[i], tofBuf + idx); // fast int -> string
          }
          else{
            tofBuf[idx++] = 'X';
          }
      }
  }
  if(sensorR.isDataReady() && sensorR.getRangingData(&measurementDataR)) {
      for(int i = 0; i < RimageResolution; i++) { 
          tofBuf[idx++] = ',';      
          const uint8_t stat  = measurementDataR.target_status[i];
          if (stat == 5){         
          idx += intToStr(measurementDataR.distance_mm[i], tofBuf + idx); // fast int -> string
          }
          else{
            tofBuf[idx++] = 'X';
          }
      }
  }
  if(sensorF.isDataReady() && sensorF.getRangingData(&measurementDataF)) {
      for(int i = 0; i < RimageResolution; i++) { 
          tofBuf[idx++] = ',';      
          const uint8_t stat  = measurementDataF.target_status[i];
          if (stat == 5){         
          idx += intToStr(measurementDataF.distance_mm[i], tofBuf + idx); // fast int -> string
          }
          else{
            tofBuf[idx++] = 'X';
          }
      }
  }
  
  tofBuf[idx++] = '\n';
  tofFile.write((uint8_t*)tofBuf, idx);  // write raw bytes
}

void logOF() {
  unsigned long now = micros();
  if (now - lastOFtime < ofInterval) return;  
  lastOFtime = now;

  if (!ofFile) return;
  // flow.readFrameBuffer(frame);
  flow.readMotionCount(&deltaX, &deltaY);
  int idx = appendTimestamp(ofBuf, now);
  // Add DeltaX and DeltaY values
  idx += snprintf(ofBuf + idx, sizeof(ofBuf) - idx, ",%d,%d\n", deltaX, deltaY);
  ofFile.write((uint8_t*)ofBuf, idx);
}


void loop() {
    // --- Handle BOOT button toggle ---
  if (digitalRead(BOOT_PIN) == LOW) {  // pressed (active LOW)
    if (millis() - lastPress > debounceDelay) {
      mode = !mode;   // toggle mode
      Serial.print("Mode toggled to: ");
      Serial.println(mode);
      lastPress = millis();
    }
  }


  if (mode) { // if mode is true, start recording
      // --- Open files once when recording starts ---
      if (!imuFile) {
        imuFile = SD.open(imuFileName, FILE_APPEND);
        tofFile = SD.open(tofFileName, FILE_APPEND);
        ofFile = SD.open(ofFileName, FILE_APPEND);


          Serial.println("Opening files for logging...");
          if (!imuFile || !tofFile || !ofFile) {
              Serial.println("Failed to open one or more files!");
          }
      }

      // --- Write data ---
      logIMU();   // writes to imuFile
      logToF();   // writes to tofFile
      logOF();    // writes to ofFile
  } 
  else {

      // --- Close files once when recording stops ---
      if (imuFile) {
          imuFile.close();
          imuFile = File(); // reset handle
          tofFile.close();
          tofFile = File();
          ofFile.close();
          ofFile = File();
          Serial.println("Files closed, safe to download.");
      }
    }
}
