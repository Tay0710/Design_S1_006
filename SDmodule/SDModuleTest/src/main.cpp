// #include <SPI.h>
// #include <SD.h>

// #define SDCS 25 // or 4 if you move CS

// void setup() {
//   Serial.begin(115200);
//   while (!SD.begin(SDCS)) {
//     Serial.println("Card Mount Failed");
//   }
//   Serial.println("Card Initialized.");

//   uint8_t cardType = SD.cardType();
//   if (cardType == CARD_NONE) {
//     Serial.println("No SD card attached");
//     return;
//   }

//   Serial.print("SD Card Type: ");
//   if (cardType == CARD_MMC) Serial.println("MMC");
//   else if (cardType == CARD_SD) Serial.println("SDSC");
//   else if (cardType == CARD_SDHC) Serial.println("SDHC");
//   else Serial.println("UNKNOWN");
// }

// void loop() {}



// #include <SPI.h>
// #include <SD.h>

// // #define HSPI_CLK 13 // SD Card, HSPI
// // #define HSPI_MISO 12
// // #define HSPI_MOSI 11
// // #define SDCS 10 //25  

// #define HSPI_CLK 12 // SD Card, HSPI
// #define HSPI_MISO 13
// #define HSPI_MOSI 11
// #define SDCS 10 //25

// SPIClass hspi(HSPI);   // SD Card


// void setup() {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("Starting SD card test with VSPI...");

//   SPI.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI);

//   while (!SD.begin(SDCS)) { // , hspi
//     Serial.println("SD init failed!");
//     delay(100);
//   }
//   Serial.println("SD card initialized.");

//   File txtFile;

//   // Keep trying until file opens
//   while (!(txtFile = SD.open("/test.txt", FILE_WRITE))) {
//       Serial.println("Failed to create test.txt, retrying...");
//       delay(500);  // wait a bit before retrying
//   }

//   // Once the file is open, write and close it
//   txtFile.println("Hello from ESP32 VSPI test");
//   txtFile.close();
//   Serial.println("test.txt created.");

//   File csvFile;

//   // Keep trying until file opens
//   while (!(csvFile = SD.open("/data.csv", FILE_WRITE))) {
//       Serial.println("Failed to create data.csv, retrying...");
//       delay(500);  // wait a bit before retrying
//   }

//   // Once the file is open, write data
//   csvFile.println("time,value");
//   csvFile.println("0,123");
//   csvFile.close();

//   Serial.println("data.csv created.");


//   Serial.println("Creation complete.");
// }

// void loop() {
//   Serial.println("Looping...");
//   delay(500);
// }



/*
STRESS TEST for SD card write performance
Writes 10,000 lines to a CSV file as fast as possible, flushing every 10ms
*/
#include <SPI.h>
#include <SD.h>
#include <Arduino.h>

// #define HSPI_CLK 13 // SD Card, HSPI
// #define HSPI_MISO 12
// #define HSPI_MOSI 11
// #define SDCS 10  

#define HSPI_CLK 13 // SD Card, HSPI
#define HSPI_MISO 12
#define HSPI_MOSI 11
#define SDCS 10 //25

#define LED1 7   // First LED on GPIO 7

SPIClass hspi(HSPI);   // SD Card

const char* fname = "/stress.csv";

const uint32_t totalLines = 100000;
uint32_t elapsed = 0;


void stressWrite() {
  // const uint32_t totalLines = 100000;
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
      Serial.print("Wrote ");
      Serial.print(i);
      Serial.println(" lines...");
    }
    // optional tiny delay to emulate sensor rate
    //delay(1);
  }

  f.close();
  elapsed = millis() - start;
  Serial.print("Wrote ");
  Serial.print(totalLines);
  Serial.print(" lines in ");
  Serial.print(elapsed);
  Serial.println(" ms");
}


void setup() {
  Serial.begin(115200);
    delay(500);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);
  delay(3000);  
  Serial.println("SD stress test starting...");
  digitalWrite(LED1, LOW);
  delay(500);


  hspi.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI);

  while (!SD.begin(SDCS, hspi, 1000000)) {
    Serial.println("SD init failed!");
  }

  Serial.println("SD initialized.");
  digitalWrite(LED1, HIGH);

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
  digitalWrite(LED1, HIGH);
  Serial.print("Wrote ");
  Serial.print(totalLines);
  Serial.print(" lines in ");
  Serial.print(elapsed);
  Serial.println(" ms");
  delay(500);
  digitalWrite(LED1, LOW);
  delay(500);

}




// /*
// LED Blink Test
// */

// #include <Arduino.h>

// #define LED1 6   // First LED on GPIO 6
// #define LED2 7   // Second LED on GPIO 7

// void setup() {
//     // Initialize pins as outputs
//     pinMode(LED1, OUTPUT);
//     pinMode(LED2, OUTPUT);
//     Serial.begin(115200);
//     delay(1000);
//     Serial.println("Starting LED blink test...");

// }

// void loop() {
//     // Turn both LEDs ON
//     digitalWrite(LED1, HIGH);
//     digitalWrite(LED2, HIGH);
//     delay(500);

//     // Turn both LEDs OFF
//     digitalWrite(LED1, LOW);
//     digitalWrite(LED2, LOW);
//     delay(500);

//     Serial.println("Blinking LEDs");
// }