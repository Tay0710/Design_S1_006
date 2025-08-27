

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>
#include <SPI.h>
#include "ICM45686.h"
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
ICM456xx IMU(SPI, 5);

int imageResolution = 0; // Used to pretty print output
int imageWidth = 0;      // Used to pretty print output

#define TRIGGER_PIN 4  // use GPIO4 as SWITCH to turn on/off when the esp32 is recording data mode. When pulled LOW, RECORDING Starts. 
bool recording = false;

SPIClass hspi(HSPI);   // Create HSPI instance
#define SD_CS 15  // Example CS pin for SD card


#define AP_SSID "ESP32_Frames"
#define AP_PASSWORD "12345678"

struct Frame {
  unsigned long timestamp;   // micros() when captured
  uint16_t distances[16];    // max 8x8 = 64 values, 4x4=16
};



// Web server
WebServer server(80);
const char* filename = "/frames.csv";



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
  IMU.startAccel(100, 16);     // 100 Hz, ±2/4/8/16/32 g
  IMU.startGyro(100, 2000);    // 100 Hz, ±15.625/31.25/62.5/125/250/500/1000/2000/4000 dps
  // Data comes out of the IMU as steps from -32768 to +32768 representing the full scale range


// Init SD card on HSPI
hspi.begin(14, 12, 13, SD_CS); // SCK=14, MISO=12, MOSI=13, CS=15
if (!SD.begin(SD_CS, hspi)) {
    Serial.println("SD init failed!");
    while(1);
}
  // Remove old file if exists
  SD.remove(filename);
  File file = SD.open(filename, FILE_WRITE);
  file.println("Frame,Timestamp(us),D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D10,D11,D12,D13,D14,D15");
  file.close();

  // Serve web page with button
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", "<html><body><h1>ESP32 Frame Download</h1><a href='/download'><button>Download CSV</button></a></body></html>");
  });

  // Serve CSV file
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

  myImager.setResolution(4*4); // Enable all 64 pads or 16 pads for 4x4 resolution

  imageResolution = myImager.getResolution(); // Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution);         // Calculate printing width

  // Using 4x4, min frequency is 1Hz and max is 60Hz
  // Using 8x8, min frequency is 1Hz and max is 15Hz
  myImager.setRangingFrequency(60);
  myImager.startRanging();

  Serial.println("Make sure the TRigger Pin is set to LOW, when starting (e.g. RESTART)");

}




void loop() {
  server.handleClient(); // handle web requests
  inv_imu_sensor_data_t imu_data; // Read IMU data
  IMU.getDataFromRegisters(imu_data);

  // Read switch state
  recording = (digitalRead(TRIGGER_PIN) == LOW); // LOW = switch ON = recording

  // Record frame to SD card if recording
  if(recording && myImager.isDataReady() && myImager.getRangingData(&measurementData)){
    File file = SD.open(filename, FILE_APPEND);
    if(file){
      file.print(micros());
      for(int i = 0; i < imageResolution; i++){
        file.print(",");
        file.print(measurementData.distance_mm[i]);
      }
      file.print(","); file.print(imu_data.accel_data[0]*16 / 32768.0); // Convert to g
      file.print(","); file.print(imu_data.accel_data[1]*16 / 32768.0);
      file.print(","); file.print(imu_data.accel_data[2]*16 / 32768.0);
      file.print(","); file.print(imu_data.gyro_data[0]*2000 / 32768.0); // Convert to dps
      file.print(","); file.print(imu_data.gyro_data[1]*2000 / 32768.0);
      file.print(","); file.print(imu_data.gyro_data[2]*2000 / 32768.0);

      file.println();
      file.close();
    }
  }
}