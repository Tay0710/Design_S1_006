#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>
#include <SPI.h>
#include "ICM45686.h"

// Use VSPI for both IMU and SD card
#define IMU_CS   5   // IMU CS pin
#define SD_CS    15  // SD card CS pin

ICM456xx IMU(SPI, IMU_CS);

#define TRIGGER_PIN 4
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
  Serial.printf("AP started. Connect to: %s\nIP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  // Initialize VSPI
  SPI.begin(18, 19, 23); // SCK=18, MISO=19, MOSI=23

  // Initialize IMU
  if (IMU.begin() != 0) {
    Serial.println("ICM456xx initialization failed");
    while (1);
  }
  IMU.startAccel(100, 16);   // 100 Hz, ±16 g
  IMU.startGyro(100, 2000);  // 100 Hz, ±2000 dps

  // Initialize SD card on same SPI bus but different CS
  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("SD init failed!");
    while (1);
  }

  // CSV header
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

  recording = (digitalRead(TRIGGER_PIN) == LOW);
  if (!recording) return;

  inv_imu_sensor_data_t imu_data;
  IMU.getDataFromRegisters(imu_data);

  File file = SD.open(filename, FILE_APPEND);
  if(file) {
    file.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
      micros(),
      imu_data.accel_data[0]*16.0/32768.0,
      imu_data.accel_data[1]*16.0/32768.0,
      imu_data.accel_data[2]*16.0/32768.0,
      imu_data.gyro_data[0]*2000.0/32768.0,
      imu_data.gyro_data[1]*2000.0/32768.0,
      imu_data.gyro_data[2]*2000.0/32768.0
    );
    file.close();
  }
}
