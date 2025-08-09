#include <Arduino.h
#include <Wire.h>
#include "platform.h"
#include <vl53l7cx_api.h>

#define SDA_PIN   21
#define SCL_PIN   44
#define SENSOR_ADDR 0x29

#define PWREN_PIN 3
#define LPN_PIN   12

VL53L7CX_Platform    platform;
VL53L7CX_Configuration dev;
VL53L7CX_ResultsData   results;

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PWREN_PIN, OUTPUT);
  digitalWrite(PWREN_PIN, HIGH);
  pinMode(LPN_PIN, OUTPUT);
  digitalWrite(LPN_PIN, HIGH);
  delay(10);

  Wire.begin(SDA_PIN, SCL_PIN);
  platform.wire    = &Wire;
  platform.address = SENSOR_ADDR;
  dev.platform     = platform;

  uint8_t alive = 0;
  if (vl53l7cx_is_alive(&dev, &alive) || !alive) {
    Serial.println("Sensor not detected!");
    while (1);
  }

  if (vl53l7cx_init(&dev)) {
    Serial.println("Init failed!");
    while (1);
  }

  dev.resolution = VL53L7CX_8X8_RESOLUTION;
  vl53l7cx_set_resolution(&dev, dev.resolution);
  vl53l7cx_start_ranging(&dev);

  Serial.println("VL53L7CX in 8x8 mode");
}

void loop() {
  uint8_t ready = 0;
  if (vl53l7cx_check_data_ready(&dev, &ready) || !ready) return;

  vl53l7cx_get_ranging_data(&dev, &results);

  for (uint8_t z = 0; z < 64; z++) {
    Serial.printf("%4d ", results.distance_mm[z]);
    if ((z + 1) % 8 == 0) Serial.println();
  }
  Serial.println();

  vl53l7cx_clear_interrupt(&dev);
  delay(100);
}