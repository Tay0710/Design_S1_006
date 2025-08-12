#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Arduino_Library.h>

VL53L5CX sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA = 21, SCL = 22
  delay(100);

  Serial.println("Starting VL53L7CX sensor...");

  if (!sensor.begin()) {
    Serial.println("Sensor initialization failed!");
    while (1);
  }

  sensor.setResolution(4, 4);  // 4x4 zones (16 zones)
  sensor.startRanging();
}

void loop() {
  if (sensor.dataReady()) {
    sensor.read();

    // Get distance from first zone (index 0)
    uint16_t distance = sensor.getDistance(0);

    Serial.print("Distance first zone (mm): ");
    Serial.println(distance);

    delay(100);
  }
}
