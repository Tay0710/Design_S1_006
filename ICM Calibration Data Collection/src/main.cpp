// Record ICM-45686 IMU data to SD card when trigger pin is LOW
// Host ESP32 serves a web page to download the CSV file

#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>
#include <SPI.h>
#include "ICM45686.h"

ICM456xx IMU(SPI, 5);  // SPI CS on GPIO5
SPIClass hspi(HSPI);   // Use HSPI for SD card

#define TRIGGER_PIN 4   // Recording switch
#define SD_CS 15        // SD card CS
#define AP_SSID "ESP32_Frames"
#define AP_PASSWORD "12345678"

bool recording = false;

// Web server
WebServer server(80);
const char* filename = "/imu.csv";

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(TRIGGER_PIN, INPUT_PULLUP);

  // Start Wi-Fi AP
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("AP started. Connect to: ");
  Serial.println(AP_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // Initialize SPI for IMU
  SPI.begin(18, 19, 23, 5); // SCK=18, MISO=19, MOSI=23, CS=5

  // Initialize IMU
  if (IMU.begin() != 0) {
    Serial.println("ICM456xx initialization failed");
    while(1);
  }
  IMU.startAccel(1600, 16);   // 100 Hz, ±16 g
  IMU.startGyro(1600, 2000);  // 100 Hz, ±2000 dps

  // Initialize SPI for SD card
  hspi.begin(14, 12, 13, SD_CS); // SCK=14, MISO=12, MOSI=13, CS=15
  if (!SD.begin(SD_CS, hspi)) {
    Serial.println("SD init failed!");
    while(1);
  }

  // Create CSV header
  SD.remove(filename);
  File file = SD.open(filename, FILE_WRITE);
  if (file) {
    file.println("Timestamp(us),AccelX(g),AccelY(g),AccelZ(g),GyroX(dps),GyroY(dps),GyroZ(dps)");
    file.close();
  }

  // Web server endpoints
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", "<html><body><h1>ESP32 IMU Download</h1><a href='/download'><button>Download CSV</button></a></body></html>");
  });

  server.on("/download", HTTP_GET, []() {
    File f = SD.open(filename);
    if(f) {
      server.streamFile(f, "text/csv");
      f.close();
    } else {
      server.send(404, "text/plain", "File not found");
    }
  });

  server.begin();
}

void loop() {
  server.handleClient();

  inv_imu_sensor_data_t imu_data;
  IMU.getDataFromRegisters(imu_data);

  recording = (digitalRead(TRIGGER_PIN) == LOW);

  if(recording) {
    File file = SD.open(filename, FILE_APPEND);
    if(file) {
      file.print(micros());
      file.print(",");
      file.print(imu_data.accel_data[0]*16.0/32768.0);
      file.print(",");
      file.print(imu_data.accel_data[1]*16.0/32768.0);
      file.print(",");
      file.print(imu_data.accel_data[2]*16.0/32768.0);
      file.print(",");
      file.print(imu_data.gyro_data[0]*2000.0/32768.0);
      file.print(",");
      file.print(imu_data.gyro_data[1]*2000.0/32768.0);
      file.print(",");
      file.print(imu_data.gyro_data[2]*2000.0/32768.0);
      file.println();
      file.close();
    }
  }
}
