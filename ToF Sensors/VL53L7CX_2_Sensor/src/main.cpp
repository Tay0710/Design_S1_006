/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642
*/

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

// LPn pins to control sensor power/reset
#define LPN 42

// I2C Reset pin
#define I2C_RST_PIN 41

// Power Enable pin
#define PWR_EN_PIN 40

#define SDA_PIN 48
#define SCL_PIN 47

// VL53L5CX default and new I2C addresses
#define SENSOR1_ADDR 0x29
#define SENSOR2_ADDR 0x30

// Sensor objects and measurement data
SparkFun_VL53L5CX sensor1;
VL53L5CX_ResultsData measurementData1; // Result data class structure, 1356 byes of RAM

SparkFun_VL53L5CX sensor2;
VL53L5CX_ResultsData measurementData2;

int imageResolution1 = 0;
int imageWidth1 = 0;

int imageResolution2 = 0;
int imageWidth2 = 0;

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
  delay(10);}
  Serial.println("SparkFun VL53L5CX Dual Sensor Example");
  
// Activating PWR_EN (Make High). 
  pinMode(PWR_EN_PIN, OUTPUT);
  digitalWrite(PWR_EN_PIN, HIGH); 
  delay(100);   

// Activating I2C Reset pin to reset the addresses (Pulse High). 
  pinMode(I2C_RST_PIN, OUTPUT);
  digitalWrite(I2C_RST_PIN, HIGH); 
  delay(100); 
  digitalWrite(I2C_RST_PIN, LOW); 
  delay(100); 


// Deactivating PWR_EN (Make Low). Reseting Sensors.  
  digitalWrite(PWR_EN_PIN, LOW); 
  delay(100);   
  digitalWrite(PWR_EN_PIN, HIGH); // Make High again.
  delay(100);  


  // Configure LPn pins
  pinMode(LPN, OUTPUT);
  // Set Sensor 1 LPn low (Deactivate I2C communication). 
  digitalWrite(LPN, LOW); // One LPn should be set HIGH permanently
  delay(100); 

  Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C bus
  Wire.setClock(400000); // Optional: 400 kHz I2C

  // Set sensor 2 to 0x30 before calling begin()
  Serial.println("Initializing Sensor 2 at 0x29 (It will become 0x30)...");

  if (!sensor2.begin()) { 
    Serial.println("Sensor 2 not found at 0x29!");
    while (1);
  }

  sensor2.setAddress(SENSOR2_ADDR);

  sensor2.setResolution(8 * 8);
  imageResolution2 = sensor2.getResolution();
  imageWidth2 = sqrt(imageResolution2);
  Serial.println("Sensor 2 initialized successfully at 0x30");

  // Sensor 1 initialization
  Serial.println("Initializing Sensor 1 at 0x29 (Stays at 0x29)...");
  // Set Sensor 1 LPn HIGH (Activate I2C communication). 
  digitalWrite(LPN, HIGH); // Other LPn should still be set HIGH
  delay(100); 

  if (!sensor1.begin()) {
    Serial.println("Sensor 1 not found at 0x29!");
    while (1);
  }

  sensor1.setResolution(8 * 8);
  imageResolution1 = sensor1.getResolution();
  imageWidth1 = sqrt(imageResolution1);
  Serial.println("Sensor 1 initialized successfully at 0x29");

  // Start ranging on both sensors. 
  Serial.println("Starting ranging on both sensors...");
  sensor1.startRanging();
  // sensor2.startRanging();

  Serial.println("Both sensors are now ranging.");
}

void loop()
{
  // Sensor 1 data ready check and read
  if (sensor1.isDataReady()) {
    if (sensor1.getRangingData(&measurementData1)) {
      Serial.println("Sensor 1 data:");
      for (int y = 0; y <= imageWidth1 * (imageWidth1 - 1); y += imageWidth1) {
        for (int x = imageWidth1 - 1; x >= 0; x--) {
          Serial.print(measurementData1.distance_mm[x + y]);
          Serial.print("\t");
        }
        Serial.println();
      }
      Serial.println();
    }
  }

  // Sensor 2 data ready check and read
  if (sensor2.isDataReady()) {
    if (sensor2.getRangingData(&measurementData2)) {
      Serial.println("Sensor 2 data:");
      for (int y = 0; y <= imageWidth2 * (imageWidth2 - 1); y += imageWidth2) {
        for (int x = imageWidth2 - 1; x >= 0; x--) {
          Serial.print(measurementData2.distance_mm[x + y]);
          Serial.print("\t");
        }
        Serial.println();
      }
      Serial.println();
    }
  }

  delay(5); // Small delay between polling
}