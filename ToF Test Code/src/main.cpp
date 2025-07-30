
#include "Adafruit_VL53L1X.h"

// https://www.st.com/resource/en/user_manual/um2356-vl53l1x-api-user-manual-stmicroelectronics.pdf#page=9&zoom=100,89,424
// Distance mode

#define IRQ_PIN 2
#define XSHUT_PIN 3
#define SDA_PIN 10
#define SCL_PIN 11

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// Currently reading up to 2m

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("Adafruit VL53L1X sensor demo"));

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin(); // SDA = 8 and SCL = 9 by default for arduino
  if (! vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Ranging started"));

  vl53.VL53L1X_SetDistanceMode(1);


  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  // Note: at 50 it measures up to 2 m ish on long range
  // on 50 ms, max of 2 m consistently
  // on 200 ms, max of 3.2 m consistently
  // on 500 ms, max of 3.5 m consistently
  vl53.setTimingBudget(500); // may need to tune this for consistency
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());


  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */
}

void loop() {
  int16_t distance;

  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(" mm");

    // data is read out, time for another reading!
    vl53.clearInterrupt();
  }
}
