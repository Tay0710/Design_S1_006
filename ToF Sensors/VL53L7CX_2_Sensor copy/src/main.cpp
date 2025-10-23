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

#define SDA_PIN 26
#define SCL_PIN 27

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

/// Reading RIGHT TOF sensor.
void readCenterAverage(SparkFun_VL53L5CX &sensor, VL53L5CX_ResultsData &measurementData)
{
  if (sensor.isDataReady())
  {
    if (sensor.getRangingData(&measurementData))
    {

      int frontSum = 0;
      int backSum = 0;
      int totalSum = 0;

      int frontCount = 0;
      int backCount = 0;
      int totalCount = 0;

      int frontAvg = 0;
      int backAvg = 0;
      int totalAvg = 0;

      int frontNorm = 0;
      int backNorm = 0;

      for (int i = 0; i < 64; i++)
      {
        uint8_t status = measurementData.target_status[i];
        int distance = measurementData.distance_mm[i];

        if (status == 5)
        { // valid return
          totalSum += distance;
          totalCount++;
          if (i < 24)
          {
            // Front 3 columns: D0 to D23
            frontSum += distance;
            frontCount++;
          }
          else if (i > 39)
          {
            // Back 3 columns: D40 to D63
            backSum += distance;
            backCount++;
          }
        }
      }

      if (totalCount > 0)
      {
        totalAvg = totalSum / totalCount;
        Serial.print("Average total distance (mm): ");
        Serial.println(totalAvg);
      }
      else
      {
        totalAvg = -1;
        Serial.println("No valid pixels.");
      }

      if (frontCount > 0)
      {
        frontAvg = frontSum / frontCount;
        Serial.print("Average front distance (mm): ");
        Serial.println(frontAvg);
      }
      else
      {
        frontAvg = -1;
        Serial.println("No valid front pixels.");
      }

      if (backCount > 0)
      {
        backAvg = backSum / backCount;
        Serial.print("Average back distance (mm): ");
        Serial.println(backAvg);
      }
      else
      {
        backAvg = -1;
        Serial.println("No valid back pixels.");
      }

      // Normalise using totalAvg
      if (totalAvg > 0) {
        frontNorm = frontAvg/totalAvg;
        Serial.print("Normalised front distance (mm): ");
        Serial.println(frontNorm);

        backNorm = backAvg/totalAvg;
        Serial.print("Normalised back distance (mm): ");
        Serial.println(backNorm);

      }
      else {
        frontNorm = -1;
        backNorm = -1;
        Serial.println("Invalid norm values");
      }
    }
    else
    {
      Serial.println("Failed to get Data!");
    }
  }
  else 
  {
    // Serial.println("Data was not ready");
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }
  Serial.println("SparkFun VL53L5CX Dual Sensor Example");

  // Activating PWR_EN (Make High).
  pinMode(PWR_EN_PIN, OUTPUT);
  digitalWrite(PWR_EN_PIN, HIGH);
  delay(200);

  // Activating I2C Reset pin to reset the addresses (Pulse High).
  pinMode(I2C_RST_PIN, OUTPUT);
  digitalWrite(I2C_RST_PIN, HIGH);
  delay(500);
  digitalWrite(I2C_RST_PIN, LOW);
  delay(500);

  // Deactivating PWR_EN (Make Low). Reseting Sensors.
  digitalWrite(PWR_EN_PIN, LOW);
  delay(500);
  digitalWrite(PWR_EN_PIN, HIGH); // Make High again.
  delay(500);

  // Configure LPn pins
  pinMode(LPN, OUTPUT);
  // Set Sensor 1 LPn low (Deactivate I2C communication).
  digitalWrite(LPN, LOW); // One LPn should be set HIGH permanently
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C bus
  Wire.setClock(400000);        // Optional: 400 kHz I2C

  // Set sensor 2 to 0x30 before calling begin()
  Serial.println("Initializing Sensor 2 at 0x29 (It will become 0x30)...");

  if (!sensor2.begin())
  {
    Serial.println("Sensor 2 not found at 0x29!");
    while (1)
      ;
  }
  delay(100);
  // sensor2.setAddress(SENSOR2_ADDR);
  delay(100);
  sensor2.setResolution(8 * 8);
  imageResolution2 = sensor2.getResolution();
  imageWidth2 = sqrt(imageResolution2);
  Serial.println("Sensor 2 initialized successfully at 0x30");

  // Sensor 1 initialization
  Serial.println("Initializing Sensor 1 at 0x29 (Stays at 0x29)...");
  // Set Sensor 1 LPn HIGH (Activate I2C communication).
  digitalWrite(LPN, HIGH); // Other LPn should still be set HIGH
  delay(100);

  if (!sensor1.begin())
  {
    Serial.println("Sensor 1 not found at 0x29!");
    while (1)
      ;
  }

  sensor1.setResolution(8 * 8);
  imageResolution1 = sensor1.getResolution();
  imageWidth1 = sqrt(imageResolution1);
  Serial.println("Sensor 1 initialized successfully at 0x29");

  // Start ranging on both sensors.
  Serial.println("Starting ranging on both sensors...");
  sensor1.startRanging();
  sensor2.startRanging();

  Serial.println("Both sensors are now ranging.");
}

void loop()
{

  readCenterAverage(sensor1, measurementData1);
  // Sensor 1 data ready check and read
  if (sensor1.isDataReady())
  {
    if (sensor1.getRangingData(&measurementData1))
    {
      Serial.println("Sensor 1 data:");
      for (int y = 0; y <= imageWidth1 * (imageWidth1 - 1); y += imageWidth1)
      {
        for (int x = imageWidth1 - 1; x >= 0; x--)
        {
          int distance = measurementData1.distance_mm[x + y];
          Serial.print(distance);
          int status = measurementData1.target_status[x + y];
          Serial.print(" s:");
          Serial.print(status);
          Serial.print("\t");
        }
        Serial.println();
      }
      Serial.println();
    }
  }

  // // Sensor 2 data ready check and read
  // if (sensor2.isDataReady()) {
  //   if (sensor2.getRangingData(&measurementData2)) {
  //     Serial.println("Sensor 2 data:");
  //     for (int y = 0; y <= imageWidth2 * (imageWidth2 - 1); y += imageWidth2) {
  //       for (int x = imageWidth2 - 1; x >= 0; x--) {
  //         Serial.print(measurementData2.distance_mm[x + y]);
  //         Serial.print("\t");
  //       }
  //       Serial.println();
  //     }
  //     Serial.println();
  //   }
  // }

  // delay(5); // Small delay between polling
}