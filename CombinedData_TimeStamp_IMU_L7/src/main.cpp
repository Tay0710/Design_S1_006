

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>
#include <SPI.h>
#include "ICM45686.h"
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
ICM456xx IMU(SPI, 5); // CS pin 5

int imageResolution = 0; // Used to pretty print output
int imageWidth = 0;      // Used to pretty print output

#define TRIGGER_PIN 4  // use GPIO4 as SWITCH to turn on/off when the esp32 is recording data mode. When pulled LOW, RECORDING Starts. 
bool recording = false;

#define SD_CS 15  // Example CS pin for SD card


#define AP_SSID "ESP32_Frames"
#define AP_PASSWORD "12345678"


// Web server
WebServer server(80);
const char* imuFile = "/imu_ICM45686.csv";
const char* tofFile = "/tof_L7.csv";

// Calibration offsets
float calibAccelX = 0, calibAccelY = 0, calibAccelZ = 0;
float calibGyroX  = 0, calibGyroY  = 0, calibGyroZ  = 0;


// Timing control
unsigned long lastIMUtime = 0;
unsigned long lastTOFtime = 0;
unsigned long lastOFtime = 0;
const unsigned long imuInterval = 2000;    // microseconds → ~500 Hz
const unsigned long tofInterval = 66000;   // microseconds → ~15 Hz
const unsigned long ofInterval = 8300;   // microseconds → ~120 Hz

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

    delay(2); // ~500 Hz
  }

  unsigned long endTime = millis();
  unsigned long duration = endTime - startTime;

  float avgAx = (float)sumAx / samples;
  float avgAy = (float)sumAy / samples;
  float avgAz = (float)sumAz / samples;
  float avgGx = (float)sumGx / samples;
  float avgGy = (float)sumGy / samples;
  float avgGz = (float)sumGz / samples;

  calibAccelX = avgAx * 16.0 / 32768.0;
  calibAccelY = avgAy * 16.0 / 32768.0;
  calibAccelZ = avgAz * 16.0 / 32768.0;
  calibGyroX  = avgGx * 2000.0 / 32768.0;
  calibGyroY  = avgGy * 2000.0 / 32768.0;
  calibGyroZ  = avgGz * 2000.0 / 32768.0;

  Serial.println("Calibration complete:");
  Serial.printf("Accel offsets: %.6f, %.6f, %.6f\n", calibAccelX, calibAccelY, calibAccelZ);
  Serial.printf("Gyro  offsets: %.6f, %.6f, %.6f\n", calibGyroX, calibGyroY, calibGyroZ);
  Serial.printf("Calibration took %lu ms (%lu samples at ~%lu Hz)\n",
                duration, samples, (samples * 1000UL) / duration);
}


void setup()
{
  int ret;

  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  pinMode(TRIGGER_PIN, INPUT_PULLUP); // pull HIGH internally


  // Start Wi-Fi AP
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("AP started. Connect to: ");
  Serial.println(AP_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

// ICM45686 Begin
  SPI.begin(18, 19, 23, 5); // SCK=18, MISO=19, MOSI=23, CS=5
  
  // --- SPI low-level WHO_AM_I test ---
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH); // CS high idle

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(5, LOW); // select IMU
  SPI.transfer(0x75 | 0x80); // 0x75 = WHO_AM_I, 0x80 = read flag
  uint8_t who_am_i = SPI.transfer(0x00); // read data
  digitalWrite(5, HIGH); // deselect
  SPI.endTransaction();
  Serial.print("WHO_AM_I = 0x");
  Serial.println(who_am_i, HEX);
  // -----------------------------------

  // Initialize the IMU using the library
  ret = IMU.begin();
  if (ret != 0) {
    Serial.print("ICM456xx initialization failed: ");
    Serial.println(ret);
    while(1);
  }

  // Configure accelerometer and gyro
  IMU.startAccel(1600, 16);     // 100 Hz, ±2/4/8/16/32 g
  IMU.startGyro(1600, 2000);    // 100 Hz, ±15.625/31.25/62.5/125/250/500/1000/2000/4000 dps
  // Data comes out of the IMU as steps from -32768 to +32768 representing the full scale range

  Serial.println("Do Not move Drone while Calibrating the ICM.");
  calibrateIMU(1000);

  // --- Initialize SD card (on same VSPI but with different CS) ---
  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("SD init failed!");
    while (1);
  }

  // Init IMU CSV
  SD.remove(imuFile);
  File imu = SD.open(imuFile, FILE_WRITE);
  imu.println("Timestamp(us),AccelX(g),AccelY(g),AccelZ(g),GyroX(dps),GyroY(dps),GyroZ(dps)");
  imu.close();

  // Init ToF CSV
  SD.remove(tofFile);
  File tof = SD.open(tofFile, FILE_WRITE);
  tof.print("Timestamp(us)");
  for(int i=0; i<16; i++) tof.print(",D"+String(i));
  tof.println();
  tof.close();

  // Serve web page with button
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html",       "<h1>ESP32 Data Logger</h1>"
      "<a href='/download_imu'><button>Download IMU CSV</button></a><br>"
      "<a href='/download_tof'><button>Download ToF CSV</button></a>");
  });

  server.on("/download_imu", HTTP_GET, []() {
    File f = SD.open(imuFile);
    if(f) { server.streamFile(f, "text/csv"); f.close(); }
    else server.send(404, "text/plain", "IMU file not found");
  });

  server.on("/download_tof", HTTP_GET, []() {
    File f = SD.open(tofFile);
    if(f) { server.streamFile(f, "text/csv"); f.close(); }
    else server.send(404, "text/plain", "ToF file not found");
  });

  server.begin();

  Wire.begin(); // This resets I2C bus to 100kHz
  Wire.setClock(1000000); //Sensor has max I2C freq of 1MHz

  Serial.println("Clock Has been Set!");

 // myImager.setWireMaxPacketSize(128); // Increase default from 32 bytes to 128 - not supported on all platforms. Default is 32 bytes. 

  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1)
      ;
  }

  myImager.setResolution(8*8); // Enable all 64 pads or 16 pads for 4x4 resolution

  imageResolution = myImager.getResolution(); // Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution);         // Calculate printing width

  // Using 4x4, min frequency is 1Hz and max is 60Hz
  // Using 8x8, min frequency is 1Hz and max is 15Hz
  myImager.setRangingFrequency(15);
  myImager.startRanging();

  Serial.println("Make sure the TRigger Pin is set to LOW, when starting (e.g. RESTART)");

}


void logIMU() {
  unsigned long now = micros();
  if (now - lastIMUtime < imuInterval) return;  
  lastIMUtime = now;

  inv_imu_sensor_data_t imu_data;
  IMU.getDataFromRegisters(imu_data);

  float ax = imu_data.accel_data[0]*16.0/32768.0 - calibAccelX;
  float ay = imu_data.accel_data[1]*16.0/32768.0 - calibAccelY;
  float az = imu_data.accel_data[2]*16.0/32768.0 - calibAccelZ;
  float gx = imu_data.gyro_data[0]*2000.0/32768.0 - calibGyroX;
  float gy = imu_data.gyro_data[1]*2000.0/32768.0 - calibGyroY;
  float gz = imu_data.gyro_data[2]*2000.0/32768.0 - calibGyroZ;

  File file = SD.open(imuFile, FILE_APPEND);
  if(file){
    file.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", now, ax, ay, az, gx, gy, gz);
    file.close();
  }
}

void logToF() {
  unsigned long now = micros();
  if (now - lastTOFtime < tofInterval) return;  
  lastTOFtime = now;

  if(myImager.isDataReady() && myImager.getRangingData(&measurementData)) {
    File file = SD.open(tofFile, FILE_APPEND);
    if(file){
      file.print(now);
      for(int i = 0; i < myImager.getResolution(); i++){
        file.print(",");
        file.print(measurementData.distance_mm[i]);
      }
      file.println();
      file.close();
    }
  }
}


void logOF() {
  unsigned long now = micros();
  if (now - lastOFtime < ofInterval) return;  
  lastOFtime = now;

  if(myImager.isDataReady() && myImager.getRangingData(&measurementData)) {
    File file = SD.open(tofFile, FILE_APPEND);
    if(file){
      file.print(now);
      for(int i = 0; i < myImager.getResolution(); i++){
        file.print(",");
        file.print(measurementData.distance_mm[i]);
      }
      file.println();
      file.close();
}


// GO FROM HERE CODE BELOW NOT COMPLETE!! 
void loop() {
  server.handleClient(); // handle web requests
  
  recording = (digitalRead(TRIGGER_PIN) == LOW);
  if (!recording) return; // If trigger is pulled HIGH, do not record data. 

  logIMU();
  logToF();
  logOF();

}