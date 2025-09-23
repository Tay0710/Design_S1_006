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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


// Example priorities (adjust based on importance)
#define IMU_TASK_PRIORITY      6
#define TOF_TASK_PRIORITY      3
#define OF_TASK_PRIORITY       6
#define SD_TASK_PRIORITY       7
// #define Idle_C0_TASK_PRIORITY  0
// #define Idle_C1_TASK_PRIORITY  0

// Stack size (adjust based on function memory usage)
#define STACK_SIZE 4096


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

// Ultrasonic inputs
#define USD 18 // Downwards-mounted ultrasonic PW pin
#define USU 15 // Upwards-mounted ultrasonic PW pin
#define USL 16 // Left-mounted ultrasonic PW pin
#define USR 4 // Right-mounted ultrasonic PW pin
#define USF 5 // Front-mounted ultrasonic PW pin

// Idle pins for monitoring core activity (optional)
// #define Idle_C0_PIN  2  // example GPIO for Core 0 idle
// #define Idle_C1_PIN  4  // example GPIO for Core 1 idle

// Boot button
#define BOOT_PIN 0

bool mode = false;         // toggled by BOOT button
unsigned long lastPress = 0;
const unsigned long debounceDelay = 200; // ms

// Mutex for SD card access
SemaphoreHandle_t sdMutex;

// VL53L7CX new I2C addresses for 2nd Sensor on I2C bus
#define CHANGE_ADDR 0x30

// Declaring the two I2C buses
// #define I2C_bus1 Wire
// #define I2C_bus2 Wire1
TwoWire I2C_bus1 = TwoWire(0); // use hardware I2C port 0
TwoWire I2C_bus2 = TwoWire(1); // use hardware I2C port 1

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
// Optical Flow and IMU on SPI
SPIClass hspi(HSPI);   // SD Card
Bitcraze_PMW3901 flow(CSOF);
ICM456xx IMU(SPI, CSIMU);

// Global file handles
File imuFile;
File ofFile;
File UltraFile;
File tofFile;
// Separate ToF files
// File tofLFile; // left
// File tofRFile; // right
// File tofUFile; // upwards
// File tofDFile; // downwards

// File Names
const char* imuFileName = "/imu_ICM45686.csv";
const char* ofFileName = "/of_PMW3901.csv";
const char* UltraFileName = "/Ultra_MB1030.csv";
const char* tofFileName = "/tof_L7.csv";
// const char* tofLFileName = "/L_tof_L7.csv";
// const char* tofRFileName = "/R_tof_L7.csv";
// const char* tofUFileName = "/U_tof_L7.csv";
// const char* tofDFileName = "/D_tof_L7.csv";

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
unsigned long lastTOFtime = 0;
// unsigned long lastTOFLtime = 0;
// unsigned long lastTOFRtime = 0;
// unsigned long lastTOFUtime = 0;
// unsigned long lastTOFDtime = 0;

const unsigned long imuInterval = 250;    // microseconds → ~4000 Hz | Low noise mode max = 6400Hz
const unsigned long ofInterval = 20000;   // microseconds → ~50 Hz
const unsigned long tofInterval = 500000/4;   // microseconds → ~4 Hz // Side Tof should be every 0.25s and Roof/ Floor ToF should be every 0.5s. 

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
    delay(1);
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
  // --- Mutex protect just the write ---
  if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
      imuFile.write((uint8_t*)imuBuf, idx);
      xSemaphoreGive(sdMutex);
  }
}

// Task: Log IMU
void imuTask(void *pvParameters) {
    for (;;) {
        logIMU();                          // your existing function
        vTaskDelay(1); // task delay for 1 ms. 
    }
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
  if (now - lastTOFtime < tofInterval) return;
  lastTOFtime = now;
  if (!tofFile) return;

  if(sensorL.isDataReady() && sensorL.getRangingData(&measurementDataL)) {
    int idx = appendTimestamp(tofLBuf, now); // write timestamp

    idx += snprintf(tofLBuf + idx, sizeof(tofLBuf) - idx, ",L");

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
  // --- Mutex protect just the write ---
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
    tofFile.write((uint8_t*)tofLBuf, idx);  // write raw bytes
    xSemaphoreGive(sdMutex);
    }
  }
}


void logToFR() {
  unsigned long now = micros();
  if (now - lastTOFtime < tofInterval) return;
  lastTOFtime = now;
  if (!tofFile) return;

  if(sensorR.isDataReady() && sensorR.getRangingData(&measurementDataR)) {
    int idx = appendTimestamp(tofRBuf, now); // write timestamp

    idx += snprintf(tofRBuf + idx, sizeof(tofRBuf) - idx, ",R");

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
    
      // --- Mutex protect just the write ---
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
      tofFile.write((uint8_t*)tofRBuf, idx);  // write raw bytes
      xSemaphoreGive(sdMutex);
    }
  }
}

void logToFU() {
  unsigned long now = micros();
  if (now - lastTOFtime < tofInterval) return;
  lastTOFtime = now;
  if (!tofFile) return;

  if(sensorU.isDataReady() && sensorU.getRangingData(&measurementDataU)) {
    int idx = appendTimestamp(tofUBuf, now); // write timestamp

    idx += snprintf(tofUBuf + idx, sizeof(tofUBuf) - idx, ",U");

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
          // --- Mutex protect just the write ---
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
      tofFile.write((uint8_t*)tofUBuf, idx);  // write raw bytes
      xSemaphoreGive(sdMutex);
    }
  }
}

void logToFD() {
  unsigned long now = micros();
  if (now - lastTOFtime < tofInterval) return;
  lastTOFtime = now;
  if (!tofFile) return;

  if(sensorD.isDataReady() && sensorD.getRangingData(&measurementDataD)) {
    int idx = appendTimestamp(tofDBuf, now); // write timestamp

    idx += snprintf(tofDBuf + idx, sizeof(tofDBuf) - idx, ",D");

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
          // --- Mutex protect just the write ---
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
      tofFile.write((uint8_t*)tofDBuf, idx);  // write raw bytes
      xSemaphoreGive(sdMutex);
    }
  }
}


volatile int tofInd = 0;

void logToF() {
  switch (tofInd) {
    case 0: logToFD(); tofInd++; break;
    case 1: logToFL(); tofInd++; break;
    case 2: logToFR(); tofInd++; break;
    case 3: logToFU(); tofInd = 0; break;
    default: tofInd = 0; break; // resets if corrupted
  }
}

// Task: Log ToF
void tofTask(void *pvParameters) {
    for (;;) {
        logToF();
        vTaskDelay(pdMS_TO_TICKS(tofInterval/1000));
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

  // --- Mutex protect just the write ---
  if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
    ofFile.write((uint8_t*)ofBuf, idx);
    xSemaphoreGive(sdMutex);
  }
}

// Task: Log Optical Flow
void ofTask(void *pvParameters) {
    for (;;) {
        logOF();
        vTaskDelay(pdMS_TO_TICKS(ofInterval/1000));
    }
}

// #define pwPin1 14
// volatile unsigned long pulseStartD = 0;
// volatile float pulseWidthD = 0;
// volatile bool usReadyD = false;

// void USD_ISR() {
//   // Called when pwPin changes
//   if (digitalRead(USD) == HIGH) {
//     // Rising edge
//     pulseStartD = micros();
//     usReadyD = false;
//   } else {
//     // Falling edge
//     unsigned long pulseWidthD = micros() - pulseStartD;
//     usReadyD = true;
//   }
// }

// // Log all 4 ultrasonic sensors (US1-US4) to CSV
// void logUltra() {
//   if (UltraFile && usReadyD) {
//     int idx = appendTimestamp(ultraBuf, pulseStartD + pulseWidthD/2);
//     float distCmD = pulseWidthD / 57.87;
//     idx += snprintf(ultraBuf + idx, sizeof(ultraBuf) - idx, ",%.2f\n", distCmD);
//     UltraFile.write((uint8_t*)ultraBuf, idx);
//   }
//   // To do: add similar if statements for 4 other ultrasonics
// }


// Task: Handle SD card open/close
void sdTask(void *pvParameters) {
    for (;;) {
        if (digitalRead(BOOT_PIN) == LOW) {  // pressed
          if(millis() - lastPress > debounceDelay){
            mode = !mode;   // toggle mode
            Serial.print("Mode toggled to: ");
            Serial.println(mode);
            lastPress = millis();   
          }
        }
      
        if(mode) {
            if (!imuFile) {
                imuFile = SD.open(imuFileName, FILE_APPEND);
                tofFile = SD.open(tofFileName, FILE_APPEND);
                ofFile = SD.open(ofFileName, FILE_APPEND);
                UltraFile = SD.open(UltraFileName, FILE_APPEND);
                Serial.println("Files opened for logging");

                if (!imuFile || !tofFile  || !ofFile || !UltraFile) {
                  Serial.println("Failed to open one or more files!");
                }
              }
            }
         else {
            if (imuFile) {  // files are open
                imuFile.close(); imuFile = File();
                tofFile.close(); tofFile = File();
                ofFile.close(); ofFile = File();
                UltraFile.close(); UltraFile = File();
                Serial.println("Files closed, safe to download");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // check BOOT pin every 50ms
    }
}


// void IdleTaskC0(void *pvParameters) {
//   pinMode(Idle_C0_PIN, OUTPUT);
//   while (1) {
//     digitalWrite(Idle_C0_PIN, HIGH);  // core is idle
//     taskYIELD();  // optional, yield to other tasks
//     digitalWrite(Idle_C0_PIN, LOW);   // brief low for measurement
//   }
// }

// void IdleTaskC1(void *pvParameters) {
//   pinMode(Idle_C1_PIN, OUTPUT);
//   while (1) {
//     digitalWrite(Idle_C1_PIN, HIGH);  // core is idle
//     taskYIELD();
//     digitalWrite(Idle_C1_PIN, LOW);   
//   }
// }


void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(BOOT_PIN, INPUT_PULLUP);
  delay(10);

  // SPI Setup
  SPI.begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI);
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
  I2C_bus1.begin(SDA1, SCL1); // This resets I2C bus to 100kHz
  I2C_bus1.setClock(1000000); //Sensor (L7) has max I2C freq of 1MHz
  delay(100);
  I2C_bus2.begin(SDA2, SCL2); 
  I2C_bus2.setClock(1000000); 
  // Serial.println(I2C_bus2.getClock());
  Serial.println("Clock Has been Set for I2Cs");

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

  // I2C bus split: L + U on I2C_bus2; R + D on I2C_bus1
  // Have to change the address of the ToFs with no LPN pin attached.
  // U + D are the sensors that have their address changed

  // Change sensor address to 0x30 after calling begin()
  if (!sensorD.begin(0x29, I2C_bus1)) { 
    Serial.println("Sensor D not found at 0x29!");
    while (1);
  } 
  Serial.println("Sensor D good");

  if (!sensorU.begin(0x29, I2C_bus2)) { 
    Serial.println("Sensor U not found at 0x29!");
    while (1);
  }
  Serial.println("Sensor U good");
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

  if (!sensorL.begin(0x29, I2C_bus1)) {
    Serial.println("Sensor L not found at 0x29!");
    while (1);
  } 
  if (!sensorR.begin(0x29, I2C_bus2)) {
    Serial.println("Sensor R not found at 0x29!");
    while (1);
  }
  sensorL.setResolution(8 * 8);
  sensorR.setResolution(8 * 8);
  LimageResolution = sensorL.getResolution();
  Serial.println("Sensors L and R initialized successfully at 0x29");
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
  pinMode(USU, OUTPUT);
  // pinMode(USL, INPUT);
  // pinMode(USR, INPUT);
  // pinMode(USF, INPUT); // USF is for object detection (front of drone). 

  digitalWrite(USU, HIGH);

  // Init IMU CSV
  SD.remove(imuFileName);
  File imu = SD.open(imuFileName, FILE_WRITE);
  imu.println("time,gyro x,gyro y,gyro z,accel x,accel y,accel z");
  imu.close();

  // Init ToF CSV/s
  SD.remove(tofFileName);
  File tof = SD.open(tofFileName, FILE_WRITE);
  tof.print("time,type");
  for(int i=0; i<((LimageResolution)); i++) tof.print(",D"+String(i));
  tof.println();
  tof.close();

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
  Ultra.print("time,distance"); 
  Ultra.println();
  Ultra.close();


  sdMutex = xSemaphoreCreateMutex();
  if (sdMutex == NULL) {
      Serial.println("Failed to create SD mutex!");
      while (1);
  }


  // Create tasks
  xTaskCreatePinnedToCore(imuTask, "IMU_Task", STACK_SIZE, NULL, IMU_TASK_PRIORITY, NULL, 1);
  xTaskCreatePinnedToCore(tofTask, "ToF_Task", STACK_SIZE, NULL, TOF_TASK_PRIORITY, NULL, 0);
  xTaskCreatePinnedToCore(ofTask, "OF_Task", STACK_SIZE, NULL, OF_TASK_PRIORITY, NULL, 1);
  xTaskCreate(sdTask, "SD_Task", STACK_SIZE, NULL, SD_TASK_PRIORITY, NULL);

  // xTaskCreatePinnedToCore(IdleTaskC0, "Idle_C0", 1024, NULL, Idle_C0_TASK_PRIORITY, NULL, 0);
  // xTaskCreatePinnedToCore(IdleTaskC1, "Idle_C1", 1024, NULL, Idle_C1_TASK_PRIORITY, NULL, 1);

  Serial.println("Finished Setup!");
  digitalWrite(USU, LOW);

}

void loop() {
  
}