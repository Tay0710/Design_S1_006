

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>
#include <SPI.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; // Used to pretty print output
int imageWidth = 0;      // Used to pretty print output

#define TRIGGER_PIN 4  // use GPIO4 as SWITCH to turn on/off when the esp32 is recording data mode. When pulled LOW, RECORDING Starts. 
bool recording = false;


#define SD_CS 5  // Example CS pin for SD card


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

  // Init SD card
  if (!SD.begin(SD_CS)) {
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
      file.println();
      file.close();
    }
  }
}