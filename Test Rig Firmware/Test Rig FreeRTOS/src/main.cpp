/*
Test Rig FreeRTOS
--------------------------
Code to sample positioning sensors and mapping sensors for both point clouds using 
FreeRTOS. This code is intended for future integration of mapping and navigation 
systems and is compatible with the Team 0-06 custom PCB and test rig.
Components: IMU, OF, SD Card, 4*ToF, 3*Ultrasonic.
*/

// ===================================================================================
// Library Imports
// ===================================================================================
#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <ICM45686.h>
#include <SparkFun_VL53L5CX_Library.h> // http://librarymanager/All#SparkFun_VL53L5CX
#include <Bitcraze_PMW3901.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ===================================================================================
// Pin Assignments
// ===================================================================================

// VSPI Pins (OF and IMU)
#define VSPI_MOSI 36
#define VSPI_CLK 37
#define VSPI_MISO 35
#define CSOF 39
#define CSIMU 38

// HSPI Pins (SD Card Module and LoRa Module)
#define HSPI_CLK 13
#define HSPI_MISO 12
#define HSPI_MOSI 11
#define SDCS 10

// I2C Bus Pins (ToFs)
#define SDA1 48   // I2C bus 1 - up and right ToFs
#define SCL1 47
#define SDA2 2    // I2C bus 2 - down and left ToFs
#define SCL2 1
#define PENA 40   // Power enable
#define RESET 41  // I2C reset
#define LPN 42    

// Ultrasonic Pins
#define USL 15    // Left-mounted ultrasonic PW pin
#define USU 18    // Upwards-mounted ultrasonic PW pin
#define USR 5     // Right-mounted ultrasonic PW pin
#define RX_PIN 16 // To pulse the UART daisy chain

// Other
#define LED1 7    // White PCB LED
#define BUTTON 4

// ===================================================================================
// RTOS Preamble
// ===================================================================================

// RTOS Task Priorities
#define IMU_TASK_PRIORITY 6
#define TOF_TASK_PRIORITY 5
#define ULTRA_TASK_PRIORITY 3
#define OF_TASK_PRIORITY 4
#define SD_TASK_PRIORITY 7


// Stack Sizes
#define STACK_SIZE 4096
#define STACK_SIZE_TOF 8192 
#define IMU_BUFFER_SIZE 37500 // 500ms @ 1 kHz, 75 bytes/sample

// Mutexes
SemaphoreHandle_t sdMutex;        // SD card module
SemaphoreHandle_t i2cBus1Mutex;   // L and D ToF sensors
SemaphoreHandle_t i2cBus2Mutex;   // R and U ToF sensors
SemaphoreHandle_t imuBufferMutex; // Protects buffer operations

// ===================================================================================
// ToF Preamble
// ===================================================================================

#define CHANGE_ADDR 0x30  // I2C addresses for the 2nd sensor on each bus

// Declaring I2C Buses
TwoWire I2C_bus1 = TwoWire(0);  // use hardware I2C port 0
TwoWire I2C_bus2 = TwoWire(1);  // use hardware I2C port 1

// Sensor Objects
SparkFun_VL53L5CX sensorL;
VL53L5CX_ResultsData measurementDataL;  // Result data class structure, 1356 byes of RAM
SparkFun_VL53L5CX sensorR;
VL53L5CX_ResultsData measurementDataR;
SparkFun_VL53L5CX sensorU;
VL53L5CX_ResultsData measurementDataU;
SparkFun_VL53L5CX sensorD;
VL53L5CX_ResultsData measurementDataD;

// Size of ToF image array
int LimageResolution = 0;   // Same for R
int DimageResolution = 0;   // Same for U

// ===================================================================================
// SD Card Preamble
// ===================================================================================

SPIClass hspi(HSPI);  // Declaring HSPI bus

// File Handles
File imuFile;
File ofFile;
File UltraFile;
File tofFile;

// Filenames
const char *imuFileName = "/imu_ICM45686.csv";
const char *ofFileName = "/of_PMW3901.csv";
const char *UltraFileName = "/Ultra_MB1030.csv";
const char *tofFileName = "/tof_L7.csv";

// Buffer Lengths
const int tofLBuf_L = 384;
const int tofRBuf_L = 384;
const int tofUBuf_L = 96;
const int tofDBuf_L = 96;
const int ultraBuf_L = 64;

// Buffers
char imuBuf[128];
char imuBufferQueue[IMU_BUFFER_SIZE];
size_t imuBufferHead = 0; // Next write position
size_t imuBufferTail = 0; // Next read position (for flushing)
char ofBuf[64];
char ultraBuf[ultraBuf_L];
char tofLBuf[tofLBuf_L];        // 8x8 resolution, with 4 character + 1 space ~ 384 chars
char tofRBuf[tofRBuf_L];
char tofUBuf[tofUBuf_L];        // 4x4 resolution ~ 96 chars (16*5 = 80)
char tofDBuf[tofDBuf_L];

// ===================================================================================
// Optical Flow Preamble
// ===================================================================================

Bitcraze_PMW3901 flow(CSOF);  // Chip select assignment (VSPI by default)

// Variables for PMW3901
char frame[35 * 35];          // Array to hold the framebuffer
int16_t deltaX, deltaY;

// ===================================================================================
// IMU Preamble
// ===================================================================================

ICM456xx IMU(SPI, CSIMU);   // Chip select assignment

// IMU - Calibration offsets
float calibAccelX = 0, calibAccelY = 0, calibAccelZ = 0;
float calibGyroX = 0, calibGyroY = 0, calibGyroZ = 0;

float G_rating = 4;     // 2/4/8/16/32 g
float dps_rating = 250; // 15.625/31.25/62.5/125/250/500/1000/2000/4000 dps

// ---- Calibration Function ----
void calibrateIMU(int samples)
{
  long sumAx = 0, sumAy = 0, sumAz = 0;
  long sumGx = 0, sumGy = 0, sumGz = 0;

  Serial.println("Starting IMU calibration...");

  unsigned long startTime = millis();

  for (int i = 0; i < samples; i++)
  {
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
  calibGyroX = avgGx * dps_rating / 32768.0;
  calibGyroY = avgGy * dps_rating / 32768.0;
  calibGyroZ = avgGz * dps_rating / 32768.0;

  Serial.println("Calibration complete:");
  Serial.printf("Accel offsets: %.6f, %.6f, %.6f\n", calibAccelX, calibAccelY, calibAccelZ);
  Serial.printf("Gyro  offsets: %.6f, %.6f, %.6f\n", calibGyroX, calibGyroY, calibGyroZ);
  Serial.printf("Calibration took %lu ms (%lu samples at ~%lu Hz)\n",
                duration, samples, (samples * 1000UL) / duration);
}

// ===================================================================================
// Ultrasonic Preamble
// ===================================================================================

volatile unsigned long pulseStartL = 0;
volatile unsigned long pulseWidthL = 0;
volatile bool usReadyL = false;
volatile unsigned long pulseStartR = 0;
volatile unsigned long pulseWidthR = 0;
volatile bool usReadyR = false;
volatile unsigned long pulseStartU = 0;
volatile unsigned long pulseWidthU = 0;
volatile bool usReadyU = false;

// ===================================================================================
// Other
// ===================================================================================

// Timing Control
unsigned long lastIMUtime = 0;
unsigned long lastOFtime = 0;
unsigned long lastTOFtime = 0;
const unsigned long imuInterval = 250;   // ~4000 Hz | Low noise mode max = 6400Hz
const unsigned long ofInterval = 18200;  // ~55Hz
const unsigned long tofInterval = 80000; // ~12.5 Hz

// Button Debounce
bool mode = false; // toggled by the button
unsigned long lastPress = 0;
const unsigned long debounceDelay = 1000; // ms

// ===================================================================================
// Helper Functions
// ===================================================================================

int appendULong(char *buf, unsigned long val)
{ // Appends unsigned long to buffer, returns number of chars
  char temp[11]; // max 10 digits + null
  int i = 0;
  if (val == 0)
  {
    buf[0] = '0';
    return 1;
  }
  while (val > 0)
  {
    temp[i++] = '0' + (val % 10);
    val /= 10;
  }
  for (int j = 0; j < i; j++)
    buf[j] = temp[i - j - 1];
  return i;
}

int appendTimestamp(char *buf, unsigned long micros_val)
{ // Writes timestamp in seconds.microseconds
  unsigned long seconds = micros_val / 1000000;
  unsigned long us = micros_val % 1000000;
  int idx = 0;
  idx += appendULong(buf + idx, seconds);
  buf[idx++] = '.';
  // Pad microseconds with leading zeros
  int digits = 6;
  char temp[6];
  for (int i = 5; i >= 0; i--)
  {
    temp[i] = '0' + (us % 10);
    us /= 10;
  }
  for (int i = 0; i < 6; i++)
    buf[idx++] = temp[i];
  return idx;
}

int intToStr(int val, char *buf, int maxLen)
{ // Fast integer to string conversion, returns number of chars written
  if (maxLen <= 0)
    return 0; // no space at all

  if (val == 0)
  {
    if (maxLen < 1)
      return 0; // need at least 1 slot
    buf[0] = '0';
    return 1;
  }

  char tmp[12]; // enough for 32-bit int
  int i = 0;

  while (val > 0 && i < (int)sizeof(tmp))
  {
    tmp[i++] = '0' + (val % 10);
    val /= 10;
  }

  if (i > maxLen)
  {
    return 0; // not enough room in destination buffer
  }

  // Copy digits in reverse order into buf
  for (int j = 0; j < i; j++)
  {
    buf[j] = tmp[i - j - 1];
  }
  return i; // number of chars written
}

uint8_t readRegister(uint8_t reg)
{ // Reads specified register of the PMW3901, returns register contents
  digitalWrite(CSOF, LOW);
  SPI.transfer(reg & 0x7F); // MSB=0 -> read
  uint8_t val = SPI.transfer(0);
  digitalWrite(CSOF, HIGH);
  return val;
}

uint16_t readShutter()
{ // Reads and combines the two shutter registers of the PMW3901
  uint8_t hi = readRegister(0x0C); // shutter high byte
  uint8_t lo = readRegister(0x0B); // shutter low byte
  return (uint16_t(hi) << 8) | lo;
}

// ===================================================================================
// Inrerrupt Service Routines
// ===================================================================================

void USL_ISR()
{ // Left ultrasonic ISR
  // Called when PW pin changes
  if (digitalRead(USL) == HIGH)
  {
    // Rising edge
    pulseStartL = micros();
  }
  else
  {
    // Falling edge
    pulseWidthL = micros() - pulseStartL;
    usReadyL = true;
  }
}

void USR_ISR()
{ // Right ultrasonic ISR
  // Called when PW pin changes
  if (digitalRead(USR) == HIGH)
  {
    // Rising edge
    pulseStartR = micros();
  }
  else
  {
    // Falling edge
    pulseWidthR = micros() - pulseStartR;
    usReadyR = true;
  }
}

void USU_ISR()
{ // Upwards ultrasonic ISR
  // Called when PW pin changes
  if (digitalRead(USU) == HIGH)
  {
    // Rising edge
    pulseStartU = micros();
  }
  else
  {
    // Falling edge
    pulseWidthU = micros() - pulseStartU;
    usReadyU = true;
  }
}

// ===================================================================================
// Data Logging Functions
// ===================================================================================

void logIMU()
{ // Logs IMU data to the IMU buffer
  if (!mode)
    return; // Skip if file is not open
  unsigned long now = micros();
  if (now - lastIMUtime < imuInterval)
    return;
  lastIMUtime = now;

  inv_imu_sensor_data_t imu_data;
  IMU.getDataFromRegisters(imu_data);

  float gx = imu_data.gyro_data[0] * dps_rating / 32768.0 - calibGyroX;
  float gy = imu_data.gyro_data[1] * dps_rating / 32768.0 - calibGyroY;
  float gz = imu_data.gyro_data[2] * dps_rating / 32768.0 - calibGyroZ;
  float ax = imu_data.accel_data[0] * G_rating / 32768.0 - calibAccelX;
  float ay = imu_data.accel_data[1] * G_rating / 32768.0 - calibAccelY;
  float az = imu_data.accel_data[2] * G_rating / 32768.0 - calibAccelZ;

  // Prepare CSV line
  char line[128];
  int len = appendTimestamp(line, now);
  len += snprintf(line + len, sizeof(line) - len, ",%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", gx, gy, gz, ax, ay, az);
  // Store in circular buffer
  if (xSemaphoreTake(imuBufferMutex, 1) == pdTRUE)
  { // 1 tick wait, non-blocking
    for (int i = 0; i < len; i++)
    {
      imuBufferQueue[imuBufferHead] = line[i];
      imuBufferHead = (imuBufferHead + 1) % IMU_BUFFER_SIZE;
      // Optional: overwrite oldest if full
      if (imuBufferHead == imuBufferTail)
      {
        imuBufferTail = (imuBufferTail + 1) % IMU_BUFFER_SIZE;
      }
    }
    xSemaphoreGive(imuBufferMutex);
  }
}

void logToFL()
{ // Logs left ToF data to the SD card
  unsigned long now = micros();
  if (!tofFile)
    return;

  if (sensorL.isDataReady() && sensorL.getRangingData(&measurementDataL))
  {
    lastTOFtime = now;

    int idx = appendTimestamp(tofLBuf, now); // write timestamp

    idx += snprintf(tofLBuf + idx, sizeof(tofLBuf) - idx, ",L");

    for (int i = 0; i < LimageResolution; i++)
    { // 8x8 = 64; 4x4 = 16
      tofLBuf[idx++] = ',';
      const uint8_t stat = measurementDataL.target_status[i];
      if (stat == 5)
      {
        idx += intToStr(measurementDataL.distance_mm[i], tofLBuf + idx, tofLBuf_L); // fast int -> string
      }
      else
      {
        tofLBuf[idx++] = 'X';
      }
    }

    tofLBuf[idx++] = '\n';
    tofFile.write((uint8_t *)tofLBuf, idx); // write raw bytes
  }
}

void logToFR()
{ // Logs right ToF data to the SD card
  unsigned long now = micros();
  if (!tofFile)
    return;

  if (sensorR.isDataReady() && sensorR.getRangingData(&measurementDataR))
  {
    lastTOFtime = now;

    int idx = appendTimestamp(tofRBuf, now); // write timestamp

    idx += snprintf(tofRBuf + idx, sizeof(tofRBuf) - idx, ",R");

    for (int i = 0; i < LimageResolution; i++)
    { // 8x8 = 64; 4x4 = 16
      tofRBuf[idx++] = ',';
      const uint8_t stat = measurementDataR.target_status[i];
      if (stat == 5)
      {
        idx += intToStr(measurementDataR.distance_mm[i], tofRBuf + idx, tofRBuf_L); // fast int -> string
      }
      else
      {
        tofRBuf[idx++] = 'X';
      }
    }

    tofRBuf[idx++] = '\n';
    tofFile.write((uint8_t *)tofRBuf, idx); // write raw bytes
  }
}

void logToFU()
{ // Logs upwards ToF data to the SD card
  unsigned long now = micros();
  if (!tofFile)
    return;

  if (sensorU.isDataReady() && sensorU.getRangingData(&measurementDataU))
  {
    lastTOFtime = now;

    int idx = appendTimestamp(tofUBuf, now); // write timestamp

    idx += snprintf(tofUBuf + idx, sizeof(tofUBuf) - idx, ",U");

    for (int i = 0; i < DimageResolution; i++)
    { // 8x8 = 64; 4x4 = 16
      tofUBuf[idx++] = ',';
      const uint8_t stat = measurementDataU.target_status[i];
      if (stat == 5)
      {
        idx += intToStr(measurementDataU.distance_mm[i], tofUBuf + idx, tofUBuf_L); // fast int -> string
      }
      else
      {
        tofUBuf[idx++] = 'X';
      }
    }

    tofUBuf[idx++] = '\n';
    tofFile.write((uint8_t *)tofUBuf, idx); // write raw bytes
  }
}

void logToFD()
{ // Logs downwards ToF data to the SD card
  unsigned long now = micros();
  if (!tofFile)
    return;

  if (sensorD.isDataReady() && sensorD.getRangingData(&measurementDataD))
  {
    lastTOFtime = now;

    int idx = appendTimestamp(tofDBuf, now); // write timestamp

    idx += snprintf(tofDBuf + idx, sizeof(tofDBuf) - idx, ",D");

    for (int i = 0; i < DimageResolution; i++)
    { // 8x8 = 64; 4x4 = 16
      tofDBuf[idx++] = ',';
      const uint8_t stat = measurementDataD.target_status[i];
      if (stat == 5)
      {
        idx += intToStr(measurementDataD.distance_mm[i], tofDBuf + idx, tofDBuf_L); // fast int -> string
      }
      else
      {
        tofDBuf[idx++] = 'X';
      }
    }

    tofDBuf[idx++] = '\n';
    tofFile.write((uint8_t *)tofDBuf, idx); // write raw bytes
  }
}

void logOF()
{ // Logs optical flow data to the SD card
  if (!mode)
    return; // if file is not open; skip!
  unsigned long now = micros();
  if (now - lastOFtime < ofInterval)
    return;
  lastOFtime = now;

  flow.readMotionCount(&deltaX, &deltaY);

  // Start buffer with timestamp
  int idx = appendTimestamp(ofBuf, now);
  uint8_t squal = readRegister(0x07);
  uint8_t shutt = readShutter();
  if (idx >= sizeof(ofBuf))
  {
    Serial.println("Buffer overflow at timestamp in logOF!");
    idx = sizeof(ofBuf) - 1;
  }
  int written = snprintf(ofBuf + idx, sizeof(ofBuf) - idx, ",%d,%d,%d,%d\n", deltaX, deltaY, squal, shutt);
  if (written <= 0 || written >= (sizeof(ofBuf) - idx))
  {
    Serial.println("Buffer overflow while writing OF data!");
    idx = sizeof(ofBuf) - 1;
  }
  else
  {
    idx += written;
  }
  // --- Mutex protect just the write ---
  if (sdMutex != NULL)
  {
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE)
    {
      if (ofFile)
        ofFile.write((uint8_t *)ofBuf, idx);
      xSemaphoreGive(sdMutex);
    }
  }
}

void logUltra()
{ // Logs ultrasonic data to the SD card
  if (!UltraFile)
    return;
  if (usReadyL)
  {
    int idx = appendTimestamp(ultraBuf, pulseStartL + pulseWidthL / 2);
    float distCmL = pulseWidthL / 57.87;
    idx += snprintf(ultraBuf + idx, sizeof(ultraBuf) - idx, ",L,%.2f\n", distCmL);
    UltraFile.write((uint8_t *)ultraBuf, idx);
    usReadyL = false;
  }
  if (usReadyR)
  {
    int idx = appendTimestamp(ultraBuf, pulseStartR + pulseWidthR / 2);
    float distCmR = pulseWidthR / 57.87;
    idx += snprintf(ultraBuf + idx, sizeof(ultraBuf) - idx, ",R,%.2f\n", distCmR);
    UltraFile.write((uint8_t *)ultraBuf, idx);
    usReadyR = false;
  }
  if (usReadyU)
  {
    int idx = appendTimestamp(ultraBuf, pulseStartU + pulseWidthU / 2);
    float distCmU = pulseWidthU / 57.87;
    idx += snprintf(ultraBuf + idx, sizeof(ultraBuf) - idx, ",U,%.2f\n", distCmU);
    UltraFile.write((uint8_t *)ultraBuf, idx);
    usReadyU = false;
  }
}

// ===================================================================================
// RTOS Tasks
// ===================================================================================

void imuTask(void *pvParameters)
{ // RTOS task to log IMU data to the IMU buffer
  for (;;)
  {
    logIMU();
    vTaskDelay(0.5);  // Task delay for 0.5 ms
  }
}

void imuFlushTask(void *pvParameters)
{ // Flushes IMU buffer onto the SD card when it is available
  for (;;)
  {
    // Only flush if SD card is available
    if (sdMutex != NULL && xSemaphoreTake(sdMutex, 10) == pdTRUE)
    {
      if (imuFile)
      {
        // Lock buffer for reading
        if (xSemaphoreTake(imuBufferMutex, portMAX_DELAY) == pdTRUE)
        {
          while (imuBufferTail != imuBufferHead)
          {
            size_t chunk;

            // Determine contiguous block size
            if (imuBufferHead > imuBufferTail)
            {
              chunk = imuBufferHead - imuBufferTail; // simple case
            }
            else
            {
              chunk = IMU_BUFFER_SIZE - imuBufferTail; // wrap-around
            }

            // Write chunk to SD
            imuFile.write((uint8_t *)&imuBufferQueue[imuBufferTail], chunk);

            // Update tail pointer
            imuBufferTail = (imuBufferTail + chunk) % IMU_BUFFER_SIZE;
          }
          xSemaphoreGive(imuBufferMutex);
        }
      }
      xSemaphoreGive(sdMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // flush every 10ms
  }
}

void tofLTask(void *pvParameters)
{ // RTOS task to log left ToF sensor
  for (;;)
  {
    logToFL();
    vTaskDelay(pdMS_TO_TICKS(tofInterval / 1000));
  }
}

void tofRTask(void *pvParameters)
{ // RTOS task to log right ToF sensor
  for (;;)
  {
    logToFR();
    vTaskDelay(pdMS_TO_TICKS(tofInterval / 1000));
  }
}

void tofUTask(void *pvParameters)
{ // RTOS task to log upwards ToF sensor
  for (;;)
  {
    logToFU();
    vTaskDelay(pdMS_TO_TICKS(tofInterval / 1000));
  }
}

void tofDTask(void *pvParameters)
{ // RTOS task to log downwards ToF sensor
  for (;;)
  {
    logToFD();
    vTaskDelay(pdMS_TO_TICKS(tofInterval / 1000));
  }
}

void ofTask(void *pvParameters)
{ // RTOS task to log optical flow camera
  for (;;)
  {
    logOF();
    vTaskDelay(pdMS_TO_TICKS(ofInterval / 1000));
  }
}

void ultraTask(void *pvParameters)
{ // RTOS task to log ultrasonic sensors
  for (;;)
  {
    logUltra();
    vTaskDelay(150);  // Task delay for 150 ms
  }
}

void sdTask(void *pvParameters)
{ // RTOS task to save data to the SD card
  for (;;)
  {
    if (digitalRead(BUTTON) == HIGH)
    { // Button pressed
      if (millis() - lastPress > debounceDelay)
      {
        mode = !mode; // toggle mode
        Serial.print("Mode toggled to: ");
        Serial.println(mode);
        lastPress = millis();
      }
    }

    if (mode)
    {
      if (!imuFile)
      {
        imuFile = SD.open(imuFileName, FILE_APPEND);
        tofFile = SD.open(tofFileName, FILE_APPEND);
        ofFile = SD.open(ofFileName, FILE_APPEND);
        UltraFile = SD.open(UltraFileName, FILE_APPEND);
        Serial.println("Files opened for logging");

        if (!imuFile || !tofFile || !ofFile || !UltraFile)
        {
          Serial.println("Failed to open one or more files!");
        }
      }
    }
    else
    {
      if (imuFile)
      { // files are open
        imuFile.close();
        imuFile = File();
        tofFile.close();
        tofFile = File();
        ofFile.close();
        ofFile = File();
        UltraFile.close();
        UltraFile = File();
        Serial.println("Files closed, safe to download");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // check BOOT pin every 50ms
  }
}

// ===================================================================================
// Setup
// ===================================================================================

void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON, INPUT_PULLDOWN);

  while (digitalRead(BUTTON) == LOW){};
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);

  // --- SPI Setup ---
  SPI.begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI);
  hspi.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI);

  // --- ICM-45686 Initialisation ---
  if (IMU.begin() != 0)
  {
    Serial.println("ICM-45686 initialisation failed");
    while (1);
  }
  Serial.println("ICM-45686 initialised.");
  IMU.startAccel(4, G_rating);      // Max 6400Hz; 100 Hz, ±2/4/8/16/32 g
  IMU.startGyro(4000, dps_rating);  // 100 Hz, ±15.625/31.25/62.5/125/250/500/1000/
  delay(100);                       // Delay needed to ensure proper calibration
  Serial.println("Do not move drone while calibrating the IMU.");
  calibrateIMU(2000);

  // --- PMW3901 Initialisation ---
  if (!flow.begin())
  {
    Serial.println("PMW3901 initialisation failed. Check wiring!");
    while (1); // stop if not found
  }
  Serial.println("PMW3901 Optical Flow sensor initialised.");
  flow.enableFrameBuffer();
  Serial.println("Frame Buffer enabled.");

  // --- SD Card Initialisation ---
  if (!SD.begin(SDCS, hspi))
  {
    Serial.println("SD init failed!");
    while (1);
  }
  Serial.println("SD Card initialised.");

  // --- ToF Initialisation ---
  // I2C bus split: U & L on I2C_bus1; D & R on I2C_bus2.
  // Have to change the address of the ToFs with LPN pulled high (U & D).

  pinMode(PENA, OUTPUT);
  digitalWrite(PENA, HIGH);
  delay(100);
  // Activating I2C Reset pin to reset the addresses (Pulse High)
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  delay(100);
  digitalWrite(RESET, LOW);
  delay(100);
  // Deactivating PWR_EN (Make Low). Reseting sensors
  digitalWrite(PENA, LOW);
  delay(100);
  digitalWrite(PENA, HIGH);
  delay(100);
  // Configure LPn pins
  pinMode(LPN, OUTPUT);
  digitalWrite(LPN, LOW); // Set sensor 1 LPn low (Deactivate I2C communication)
  Serial.println("Successfully reset ToF sensors' I2C addresses.");

  I2C_bus1.begin(SDA1, SCL1); 
  I2C_bus1.setClock(400000);  // VL53L7CX has max I2C freq of 1MHz
  I2C_bus2.begin(SDA2, SCL2);
  I2C_bus2.setClock(400000);
  Serial.println("Clock Has been Set for I2Cs");

  if (!sensorU.begin(0x29, I2C_bus1))
  {
    Serial.println("ToF U not found at 0x29!");
    while (1)
      ;
  }
  Serial.println("ToF U initialised");
  if (!sensorD.begin(0x29, I2C_bus2))
  {
    Serial.println("ToF D not found at 0x29!");
    while (1)
      ;
  }
  Serial.println("ToF D initialised");

  sensorU.setAddress(CHANGE_ADDR);  // Change sensor address to 0x30
  sensorD.setAddress(CHANGE_ADDR);
  sensorU.setResolution(4 * 4);
  sensorD.setResolution(4 * 4);
  DimageResolution = sensorD.getResolution();
  Serial.println("ToFs U & D initialised successfully at 0x30");

  // Default Address Sensor initialisation
  digitalWrite(LPN, HIGH); // Set Sensor LPn HIGH (Activate I2C communication).
  delay(100);

  if (!sensorL.begin(0x29, I2C_bus1))
  {
    Serial.println("ToF L not found at 0x29!");
    while (1);
  }
  Serial.println("ToF L initialised");
  if (!sensorR.begin(0x29, I2C_bus2))
  {
    Serial.println("ToF R not found at 0x29!");
    while (1);
  }
  Serial.println("ToF R initialised");

  sensorL.setResolution(8 * 8);
  sensorR.setResolution(8 * 8);
  LimageResolution = sensorL.getResolution();

  Serial.println("ToFs L & R initialised successfully at 0x29");

  // Set frequency & start ranging
  sensorU.setRangingFrequency(15);
  sensorD.setRangingFrequency(15);
  sensorL.setRangingFrequency(15);
  sensorR.setRangingFrequency(15);
  sensorU.startRanging();
  delay(50);
  sensorD.startRanging();
  delay(50);
  sensorL.startRanging();
  delay(50);
  sensorR.startRanging();
  delay(50);
  Serial.println("ToFs are ranging.");

  // --- Ultrasonic Initialisation ---
  pinMode(USL, INPUT);
  pinMode(USR, INPUT);
  pinMode(USU, INPUT);
  attachInterrupt(digitalPinToInterrupt(USL), USL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(USR), USR_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(USU), USU_ISR, CHANGE);
  Serial.println("Ultrasonics initialised successfully");

  // --- CSV Initialisation ---
  // Init IMU CSV
  SD.remove(imuFileName);
  File imu = SD.open(imuFileName, FILE_WRITE);
  imu.println("time,gyro x,gyro y,gyro z,accel x,accel y,accel z");
  imu.close();

  // Init ToF CSV/s
  SD.remove(tofFileName);
  File tof = SD.open(tofFileName, FILE_WRITE);
  tof.print("time,type");
  for (int i = 0; i < ((LimageResolution)); i++)
    tof.print(",D" + String(i));
  tof.println();
  tof.close();

  // Init OF CSV
  SD.remove(ofFileName);
  File of = SD.open(ofFileName, FILE_WRITE);
  of.print("time");
  of.print(",deltaX,deltaY,SQUAL,SHUTTER");
  of.println();
  of.close();

  // Init Ultra CSVs
  SD.remove(UltraFileName);
  File Ultra = SD.open(UltraFileName, FILE_WRITE);
  Ultra.print("time,distance");
  Ultra.println();
  Ultra.close();

  // --- Mutex Preparation ---
  // Create SD card mutex
  sdMutex = xSemaphoreCreateMutex();
  if (sdMutex == NULL)
  {
    Serial.println("Failed to create SD mutex!");
    while (1)
      ;
  }

  // Create I2C mutexes
  i2cBus1Mutex = xSemaphoreCreateMutex();
  i2cBus2Mutex = xSemaphoreCreateMutex();
  if (i2cBus1Mutex == NULL || i2cBus2Mutex == NULL)
  {
    Serial.println("Failed to create I2C mutex!");
    while (1)
      ; // halt
  }

  // Create IMU buffer mutex - prevents concurrent access to imuBuffer
  imuBufferMutex = xSemaphoreCreateMutex();
  if (imuBufferMutex == NULL)
  {
    Serial.println("Failed to create IMU buffer mutex!");
    while (1)
      ;
  }

  // --- RTOS Task Creation ---
  xTaskCreatePinnedToCore(imuTask, "IMU_Task", STACK_SIZE, NULL, IMU_TASK_PRIORITY, NULL, 1);
  xTaskCreate(imuFlushTask, "IMU_Flush", 8192, NULL, IMU_TASK_PRIORITY, NULL);
  xTaskCreatePinnedToCore(ofTask, "OF_Task", STACK_SIZE, NULL, OF_TASK_PRIORITY, NULL, 1);
  xTaskCreate(sdTask, "SD_Task", STACK_SIZE, NULL, SD_TASK_PRIORITY, NULL);
  xTaskCreatePinnedToCore(tofLTask, "ToF_L", STACK_SIZE_TOF, NULL, TOF_TASK_PRIORITY, NULL, 0);
  xTaskCreatePinnedToCore(tofRTask, "ToF_R", STACK_SIZE_TOF, NULL, TOF_TASK_PRIORITY, NULL, 0);
  xTaskCreatePinnedToCore(tofUTask, "ToF_U", STACK_SIZE_TOF, NULL, TOF_TASK_PRIORITY, NULL, 0);
  xTaskCreatePinnedToCore(tofDTask, "ToF_D", STACK_SIZE_TOF, NULL, TOF_TASK_PRIORITY, NULL, 0);

  Serial.println("Finished Setup!");

  digitalWrite(LED1, LOW); // Turning off LED to indicate complete setup

  // --- Pulsing Ultrasonics ---
  // Pulse required to initialise UART daisy chain
  digitalWrite(LED1, LOW);
  pinMode(RX_PIN, OUTPUT);
  digitalWrite(RX_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(RX_PIN, LOW);
  pinMode(RX_PIN, INPUT);
}

void loop()
{
}