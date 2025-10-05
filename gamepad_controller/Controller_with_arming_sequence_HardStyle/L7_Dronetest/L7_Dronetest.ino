#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

// // LPn pins to control sensor power/reset
// #define LPN_PIN_1 42

// // I2C Reset pin
// #define I2C_RST_PIN 41

// // Power Enable pin
// #define PWR_EN_PIN 40

#define SDA_PIN 26
#define SCL_PIN 27

 // AVDD - needs 3v3 supply
 // IOVDD - needs 3v3 supply
 // GND - GND

//  #define P1 19 // LPn - not required. 



// Sensor objects and measurement data
SparkFun_VL53L5CX sensor1;
VL53L5CX_ResultsData measurementData1; // Result data class structure, 1356 byes of RAM


int imageResolution1 = 0;
int imageWidth1 = 0;







void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Dual Sensor Example");

  // pinMode(P1, OUTPUT); // 
  // digitalWrite(P1, HIGH); // 
  // delay(100);
  // pinMode(P2, OUTPUT); // 
  // digitalWrite(P2, HIGH); // 
  // delay(100);
  // pinMode(P3, OUTPUT); // GND
  // digitalWrite(P3, LOW); // 
  // delay(100);
  

  Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C bus
  Wire.setClock(400000); // Optional: 400 kHz I2C
  delay(50);

  if (!sensor1.begin()) {
    Serial.println("Sensor 1 not found at 0x29!");
    while (1);
  }

  sensor1.setResolution(4 * 4);
  imageResolution1 = sensor1.getResolution();
  imageWidth1 = sqrt(imageResolution1);
  Serial.println("Sensor 1 initialized successfully at 0x29");

  delay(50);

  // Start ranging on both sensors. 
  Serial.println("Starting ranging on both sensors...");
  sensor1.setRangingFrequency(60);
  sensor1.startRanging();
  Serial.println("Both sensors are now ranging.");

}



void loop()
{
  if (sensor1.isDataReady()) {
    if (sensor1.getRangingData(&measurementData1)) {
      
      // Middle 4 indices in 4x4 grid: 5, 6, 9, 10
      int middleIdx[4] = {5, 6, 9, 10};  // can get all points by using: sensor1.getResolution(); returns 16 for 4*4 array. 
      int sum = 0;
      int count = 0;

      for (int i = 0; i < 4; i++) {
        int idxPixel = middleIdx[i];
        uint8_t status = measurementData1.target_status[idxPixel];

        if (status == 5) { // valid return
          sum += measurementData1.distance_mm[idxPixel];
          count++;
        }
      }

      if (count > 0) {
        int avg = sum / count;
        Serial.print("Average center distance (mm): ");
        Serial.println(avg);
      } else {
        Serial.println("No valid center pixels.");
      }
    }
  }

  delay(5); // Small delay between polling
}