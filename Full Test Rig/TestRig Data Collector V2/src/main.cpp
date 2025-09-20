// 

// Components:
// IMU, OF, SD Card, 4*ToF, 5*Ultrasonic (4 mapping, 1 obejct detection)
// Missing SBUS commands

#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <ICM45686.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <Bitcraze_PMW3901.h>

// Optical flow SPI pins
#define VSPI_MOSI 36 // OF and IMU, VSPI
#define VSPI_CLK 37
#define VSPI_MISO 35
#define CSOF 39
#define CSIMU 38

#define HSPI_CLK 13 // SD Card, HSPI
#define HSPI_MISO 12
#define HSPI_MOSI 11
#define SDCS 10

// ToF inputs
#define SDA1 48  
#define SCL1 47  
#define SDA2 2
#define SCL2 1
#define PENA 40 // Power enable
#define RESET 41 // I2C reset
#define LPN 42

#define USD 18 // Downwards-mounted ultrasonic PW pin
#define USU 15 // Upwards-mounted ultrasonic PW pin
#define USL 16 // Left-mounted ultrasonic PW pin
#define USR 4 // Right-mounted ultrasonic PW pin
#define USF 5 // Front-mounted ultrasonic PW pin

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
SparkFun_VL53L5CX sensorL;
VL53L5CX_ResultsData measurementDataL; // Result data class structure, 1356 byes of RAM
SparkFun_VL53L5CX sensorR;
VL53L5CX_ResultsData measurementDataR;
SparkFun_VL53L5CX sensorU;
VL53L5CX_ResultsData measurementDataU;
SparkFun_VL53L5CX sensorD;
VL53L5CX_ResultsData measurementDataD;

// Chip select assignment
// Create SPI buses
SPIClass vspi(VSPI);   // Optical Flow and IMU
SPIClass hspi(HSPI);   // SD Card
Bitcraze_PMW3901 flow(CSOF);
ICM456xx IMU(VSPI, CSIMU);

// Global file handles
File imuFile;
File ofFile;
File UltraFile;
// Separate ToF files
File tofLFile; // left
File tofRFile; // right
File tofUFile; // upwards
File tofDFile; // downwards

// File Names
const char* imuFileName = "/imu_ICM45686.csv";
const char* ofFileName = "/of_PMW3901.csv";
const char* UltraFileName = "/Ultra_MB1030.csv";
const char* tofLFileName = "/L_tof_L7.csv";
const char* tofRFileName = "/R_tof_L7.csv";
const char* tofUFileName = "/U_tof_L7.csv";
const char* tofDFileName = "/D_tof_L7.csv";

// Variables for PMW3901
char frame[35*35]; //array to hold the framebuffer
int16_t deltaX, deltaY;

// Size of ToF image array
int LimageResolution = 0; // Same for R
int UimageResolution  = 0; // Same for D

// Char buffers
char imuBuf[128];
char ofBuf[64];
char ultraBuf[64];
// char ultraBuf[128]; // enough for timestamp + 4 readings
char tofLBuf[384]; // 8x8 resolution, with 4 character + 1 space ~ 384 chars
char tofRBuf[384]; 
char tofUBuf[96]; // 4x4 resolution ~ 96 chars (16*5 = 80)
char tofDBuf[96];
// char tofBuf[1024]; // All 4 ToFs in one buffer

// IMU - Calibration offsets
float calibAccelX = 0, calibAccelY = 0, calibAccelZ = 0;
float calibGyroX  = 0, calibGyroY  = 0, calibGyroZ  = 0;

float  G_rating = 4;      // 2/4/8/16/32 g
float  dps_rating = 250; // 15.625/31.25/62.5/125/250/500/1000/2000/4000 dps

// Timing control
unsigned long lastIMUtime = 0;
unsigned long lastOFtime = 0;
unsigned long lastTOFLtime = 0;
unsigned long lastTOFRtime = 0;
unsigned long lastTOFUtime = 0;
unsigned long lastTOFDtime = 0;
const unsigned long imuInterval = 250;    // microseconds → ~4000 Hz | Low noise mode max = 6400Hz
const unsigned long ofInterval = 20000;   // microseconds → ~50 Hz
const unsigned long tofInterval = int(250000/4);   // microseconds → ~4 Hz // Side Tof should be every 0.25s and Roof/ Floor ToF should be every 0.5s. 

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
  if (!imuFile) return; // if file is not open; skip!

  inv_imu_sensor_data_t imu_data;
  IMU.getDataFromRegisters(imu_data);

  float gx = imu_data.gyro_data[0]*dps_rating/32768.0 - calibGyroX; // To do - confirm if max is 32767 or 32768
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

void logToFL() {
  unsigned long now = micros();
  if (!tofLFile) return;

  if(sensorL.isDataReady() && sensorR.getRangingData(&measurementDataL)) {
    int idx = appendTimestamp(tofLBuf, now); // write timestamp

    for(int i = 0; i < LimageResolution; i++) { //8x8 = 64; 4x4 = 16
      tofLBuf[idx++] = ',';      
      const uint8_t stat  = measurementDataL.target_status[i];
      if (stat == 5){         
      idx += intToStr(measurementDataL.distance_mm[i], tofLBuf + idx); // fast int -> string
      }
      else{
        tofLBuf[idx++] = 'X';
      }
    }

    tofLBuf[idx++] = '\n';
    tofLFile.write((uint8_t*)tofLBuf, idx);  // write raw bytes
  }
}

void logToFR() {
  unsigned long now = micros();
  if (!tofRFile) return;

  if(sensorR.isDataReady() && sensorR.getRangingData(&measurementDataR)) {
    int idx = appendTimestamp(tofRBuf, now); // write timestamp

    for(int i = 0; i < LimageResolution; i++) { //8x8 = 64; 4x4 = 16
      tofRBuf[idx++] = ',';      
      const uint8_t stat  = measurementDataR.target_status[i];
      if (stat == 5){         
      idx += intToStr(measurementDataR.distance_mm[i], tofRBuf + idx); // fast int -> string
      }
      else{
        tofRBuf[idx++] = 'X';
      }
    }

    tofRBuf[idx++] = '\n';
    tofRFile.write((uint8_t*)tofRBuf, idx);  // write raw bytes
  }
}

void logToFU() {
  unsigned long now = micros();
  if (!tofUFile) return;

  if(sensorU.isDataReady() && sensorU.getRangingData(&measurementDataU)) {

    int idx = appendTimestamp(tofUBuf, now); // write timestamp

    for(int i = 0; i < UimageResolution; i++) { //8x8 = 64; 4x4 = 16
      tofUBuf[idx++] = ',';      
      const uint8_t stat  = measurementDataU.target_status[i];
      if (stat == 5){         
      idx += intToStr(measurementDataU.distance_mm[i], tofUBuf + idx); // fast int -> string
      }
      else{
        tofUBuf[idx++] = 'X';
      }
    }

    tofUBuf[idx++] = '\n';
    tofUFile.write((uint8_t*)tofUBuf, idx);  // write raw bytes
  }
}

void logToFD() {
  unsigned long now = micros();
  if (!tofRFile) return;

  if(sensorD.isDataReady() && sensorD.getRangingData(&measurementDataD)) {

    int idx = appendTimestamp(tofDBuf, now); // write timestamp

    for(int i = 0; i < UimageResolution; i++) { //8x8 = 64; 4x4 = 16
      tofDBuf[idx++] = ',';      
      const uint8_t stat  = measurementDataD.target_status[i];
      if (stat == 5){         
      idx += intToStr(measurementDataD.distance_mm[i], tofDBuf + idx); // fast int -> string
      }
      else{
        tofDBuf[idx++] = 'X';
      }
    }

    tofDBuf[idx++] = '\n';
    tofDFile.write((uint8_t*)tofDBuf, idx);  // write raw bytes
  }
}

volatile int tofInd = 0;

void logToF() {
  unsigned long now = micros();
  if (now - lastTOFtime < tofInterval) return;
  lastTOFtime = now;

  switch (tofInd) {
    case 0: logToFD(); tofInd++;
    case 1: logToFL(); tofInd++;
    case 2: logToFR(); tofInd++;
    case 3: logToFU(); tofInd = 0;
  }
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

#define pwPin1 14
volatile unsigned long pulseStartD = 0;
volatile float pulseWidthD = 0;
volatile bool usReadyD = false;

void USD_ISR() {
  // Called when pwPin changes
  if (digitalRead(USD) == HIGH) {
    // Rising edge
    pulseStartD = micros();
    usReadyD = false;
  } else {
    // Falling edge
    unsigned long pulseWidthD = micros() - pulseStartD;
    usReadyD = true;
  }
}

// Log all 4 ultrasonic sensors (US1-US4) to CSV
void logUltra() {
  if (UltraFile && usReadyD) {
    int idx = appendTimestamp(ultraBuf, pulseStartD + pulseWidthD/2);
    float distCmD = pulseWidthD / 57.87;
    idx += snprintf(ultraBuf + idx, sizeof(ultraBuf) - idx, ",%.2f\n", distCmD);
    UltraFile.write((uint8_t*)ultraBuf, idx);
  }
  // To do: add similar if statements for 4 other ultrasonics
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(BOOT_PIN, INPUT_PULLUP);
  delay(10);

  // SPI Setup
  vspi.begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI);
  hspi.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI);

  // Initialise ICM45686 - using vspi.
  if (IMU.begin() != 0) {
    Serial.println("ICM456xx initialization failed");
    while (1);
  }
  // Configure accelerometer and gyro
  IMU.startAccel(1600, G_rating);     // Max 6400Hz; 100 Hz, ±2/4/8/16/32 g
  IMU.startGyro(1600, dps_rating);    // 100 Hz, ±15.625/31.25/62.5/125/250/500/1000/
  delay(100); // Delay needed to ensure proper calibration
  Serial.println("Do not move drone while calibrating the ICM.");
  calibrateIMU(2000);

  // Initialise PMW3901 - using vspi
  if (!flow.begin()) {
      Serial.println("PMW3901 initialization failed. Check wiring!");
      while (1);  // stop if not found
  }
  delay(100);
  flow.enableFrameBuffer(); 

  // Initialise SD card - using hspi
  if (!SD.begin(SDCS, hspi)) {
    Serial.println("SD init failed!");
    while (1);
  }

  // Initialise 4 ToF sensors
  I2C1.begin(); // This resets I2C bus to 100kHz
  I2C1.setClock(1000000); //Sensor (L7) has max I2C freq of 1MHz
  I2C2.begin(); 
  I2C2.setClock(1000000); 
  Serial.println("Clock Has been Set for I2C's!");

  // Address reset sequence for ToFL7 sensors.
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
  // Deactivating PWR_EN (Make Low). Reseting sensors
  digitalWrite(PENA, LOW); 
  delay(100);   
  digitalWrite(PENA, HIGH); // Make high again
  delay(100);  
  // Configure LPn pins
  pinMode(LPN, OUTPUT);
  // Set sensor 1 LPn low (Deactivate I2C communication) 
  digitalWrite(LPN, LOW); // One LPn should be set HIGH permanently
  delay(100); 

  // I2C bus split: L + U on I2C1; R + D on I2C2
  // Have to change the address of the ToFs with no LPN pin attached.
  // U + D are the sensors that have their address changed

  // Change sensor address to 0x30 after calling begin()
    if (!sensorU.begin()) { 
    Serial.println("Sensor U not found at 0x29!");
    while (1);
  }
    if (!sensorD.begin()) { 
    Serial.println("Sensor D not found at 0x29!");
    while (1);
  }
  sensorU.setAddress(CHANGE_ADDR);
  sensorD.setAddress(CHANGE_ADDR);
  sensorU.setResolution(4 * 4);
  sensorD.setResolution(4 * 4);
  // Using 4x4, min frequency is 1Hz and max is 60Hz
  // Using 8x8, min frequency is 1Hz and max is 15Hz
  UimageResolution = sensorU.getResolution();
  Serial.println("Sensor's initialized successfully at 0x30");
  delay(50);

  // Default Address Sensor initialization - Set Sensor LPn HIGH (Activate I2C communication). 
  digitalWrite(LPN, HIGH); // Other LPn should still be set HIGH
  delay(100); 

  if (!sensorL.begin()) {
    Serial.println("Sensor L not found at 0x29!");
    while (1);
  } 
  if (!sensorR.begin()) {
    Serial.println("Sensor R not found at 0x29!");
    while (1);
  }
  sensorL.setResolution(8 * 8);
  sensorR.setResolution(8 * 8);
  LimageResolution = sensorL.getResolution();
  Serial.println("Sensor 1 initialized successfully at 0x29");
  delay(50);

  // // Set Frequency
  // sensorR.setRangingFrequency(15);
  // sensorF.setRangingFrequency(15);
  // sensorS1.setRangingFrequency(15);
  // sensorS2.setRangingFrequency(15);
  // Start ranging on both sensors. 
  Serial.println("Starting ranging of ToFL7  sensors...");
  sensorU.startRanging();
  sensorD.startRanging();
  sensorL.startRanging();
  sensorR.startRanging();
  Serial.println("Sensors are now ranging.");

  // Ultrasonics
  pinMode(USD, INPUT); // Note: Ultrasonics operate on a 49mS cycle.
  // pinMode(USU, INPUT);
  // pinMode(USL, INPUT);
  // pinMode(USR, INPUT);
  // pinMode(USF, INPUT); // USF is for object detection (front of drone). 

  // Init IMU CSV
  SD.remove(imuFileName);
  File imu = SD.open(imuFileName, FILE_WRITE);
  imu.println("time,gyro x,gyro y,gyro z,accel x,accel y,accel z");
  imu.close();

  // Init ToF CSVs
  SD.remove(tofUFileName);
  File tofU = SD.open(tofUFileName, FILE_WRITE);
  tofU.print("time,type");
  for(int i=0; i<((UimageResolution)); i++) tofU.print(",D"+String(i));
  tofU.println();
  tofU.close();
  SD.remove(tofDFileName);
  File tofD = SD.open(tofDFileName, FILE_WRITE);
  tofD.print("time,type");
  for(int i=0; i<((UimageResolution)); i++) tofD.print(",D"+String(i));
  tofD.println();
  tofD.close();
  SD.remove(tofLFileName);
  File tofL = SD.open(tofLFileName, FILE_WRITE);
  tofL.print("time,type");
  for(int i=0; i<((LimageResolution)); i++) tofL.print(",D"+String(i));
  tofL.println();
  tofL.close();
  SD.remove(tofRFileName);
  File tofR = SD.open(tofRFileName, FILE_WRITE);
  tofR.print("time,type");
  for(int i=0; i<((LimageResolution)); i++) tofR.print(",D"+String(i));
  tofR.println();
  tofR.close();

  // Init OF CSV
  SD.remove(ofFileName);
  File of = SD.open(ofFileName, FILE_WRITE);
  of.print("time");
  of.print(",deltaX,deltaY");
  of.println();
  of.close();

  // Init Ultra CSVs
  SD.remove(UltraFileName);
  File Ultra = SD.open(UltraFileName, FILE_WRITE);
  Ultra.print("time,US1,US2,US3,US4"); 
  Ultra.println();
  Ultra.close();

  Serial.println("Finished Setup!");
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
      tofUFile = SD.open(tofUFileName, FILE_APPEND);
      tofDFile = SD.open(tofDFileName, FILE_APPEND);
      tofLFile = SD.open(tofLFileName, FILE_APPEND);
      tofRFile = SD.open(tofRFileName, FILE_APPEND);
      ofFile = SD.open(ofFileName, FILE_APPEND);
      UltraFile = SD.open(UltraFileName, FILE_APPEND);

      Serial.println("Opening files for logging...");
      if (!imuFile || !tofUFile  || !tofDFile|| !tofLFile|| !tofRFile || !ofFile || !UltraFile) {
          Serial.println("Failed to open one or more files!");
      }
    }

    // --- Write data ---
    logIMU();   // writes to imuFile
    logToFL();   // writes to tofLFile
    logToFR();   // writes to tofRFile
    logToFU();   // writes to tofUFile
    logToFD();   // writes to tofDFile
    logOF();    // writes to ofFile
    logUltra();
  } 
  else {
    // --- Close files once when recording stops ---
    if (imuFile) {
      imuFile.close();
      imuFile = File(); // reset handle
      tofUFile.close();
      tofUFile = File();
      tofDFile.close();
      tofDFile = File();
      tofLFile.close();
      tofLFile = File();
      tofRFile.close();
      tofRFile = File();
      ofFile.close();
      ofFile = File();
      UltraFile.close();
      UltraFile = File();
      Serial.println("Files closed, safe to download.");
    }
  }
}
