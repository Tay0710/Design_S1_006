// #include <SPI.h>
// #include <SD.h>

// // VSPI default pins: SCK=18, MISO=19, MOSI=23
// #define SD_CS 5   // Chip Select pin (safe choice)

// void setup() {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("Starting SD card test with VSPI...");

//   // Init SD on VSPI (no need for custom SPIClass, defaults are used)
//   if (!SD.begin(SD_CS)) {
//     Serial.println("SD init failed!");
//     while (true);
//   }
//   Serial.println("SD card initialized.");

//   // Create TXT file
//   File txtFile = SD.open("/test.txt", FILE_WRITE);
//   if (txtFile) {
//     txtFile.println("Hello from ESP32 VSPI test");
//     txtFile.close();
//     Serial.println("test.txt created.");
//   } else {
//     Serial.println("Failed to create test.txt");
//   }

//   // Create CSV file
//   File csvFile = SD.open("/data.csv", FILE_WRITE);
//   if (csvFile) {
//     csvFile.println("time,value");
//     csvFile.println("0,123");
//     csvFile.close();
//     Serial.println("data.csv created.");
//   } else {
//     Serial.println("Failed to create data.csv");
//   }

//   Serial.println("Creation complete.");
// }

// void loop() {
//   // nothing else to do
// }



/*
STRESS TEST for SD card write performance
Writes 10,000 lines to a CSV file as fast as possible, flushing every 10ms
*/
#include <SPI.h>
#include <SD.h>

#define SD_CS 5   // change if needed
const char* fname = "/stress.csv";

void stressWrite() {
  const uint32_t totalLines = 100000;
  const uint32_t flushEvery = 10000; // tune this
  uint32_t start = millis();

  File f = SD.open(fname, FILE_APPEND);
  if (!f) {
    Serial.println("Failed to open file for append");
    return;
  }

  for (uint32_t i = 0; i < totalLines; i++) {
    uint32_t t = millis();
    // example data: idx, time, pseudo-value
    f.printf("%lu,%lu,%d\n", i, t, (int)(i % 100));
    if ((i % flushEvery) == 0) {
      f.flush(); // pushes to card; increases chance to show errors
      // Serial.print("Wrote ");
      // Serial.print(i);
      // Serial.println(" lines...");
    }
    // optional tiny delay to emulate sensor rate
    //delay(1);
  }

  f.close();
  uint32_t elapsed = millis() - start;
  Serial.print("Wrote ");
  Serial.print(totalLines);
  Serial.print(" lines in ");
  Serial.print(elapsed);
  Serial.println(" ms");
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("SD stress test starting...");

  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed!");
    while (true);
  }
  Serial.println("SD initialized.");

  // create/overwrite file with header
  File f = SD.open(fname, FILE_WRITE);
  if (!f) { Serial.println("Failed to open file"); while(true); }
  f.println("idx,timestamp_ms,value");
  f.close();

  delay(200);
  stressWrite();
  Serial.println("Test finished.");
}

void loop() {
  // nothing
}