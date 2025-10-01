#include <SPI.h>
#include <SD.h>

// VSPI default pins: SCK=18, MISO=19, MOSI=23
#define SD_CS 5   // Chip Select pin (safe choice)

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting SD card test with VSPI...");

  // Init SD on VSPI (no need for custom SPIClass, defaults are used)
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed!");
    while (true);
  }
  Serial.println("SD card initialized.");

  // Create TXT file
  File txtFile = SD.open("/test.txt", FILE_WRITE);
  if (txtFile) {
    txtFile.println("Hello from ESP32 VSPI test");
    txtFile.close();
    Serial.println("test.txt created.");
  } else {
    Serial.println("Failed to create test.txt");
  }

  // Create CSV file
  File csvFile = SD.open("/data.csv", FILE_WRITE);
  if (csvFile) {
    csvFile.println("time,value");
    csvFile.println("0,123");
    csvFile.close();
    Serial.println("data.csv created.");
  } else {
    Serial.println("Failed to create data.csv");
  }

  Serial.println("Creation complete.");
}

void loop() {
  // nothing else to do
}
