#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> // http://librarymanager/All#SparkFun_VL53L5CX

// ----- Sensor objects -----
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // ~1356 bytes

// ----- Presentation settings -----
int imageResolution = 0; // 16 for 4x4, 64 for 8x8
int imageWidth      = 0; // 4 or 8

// Adjust this if you want a different definition of "valid"
static inline bool isValidStatus(uint8_t s) {
  // ST docs: 5 is the canonical "valid" target. You can expand this if needed.
  return (s == 5);
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println(F("\nVL53L5CX: full results (per-zone and per-target)"));

  Wire.begin();                 // default SDA/SCL (ESP32: 21/22 unless changed)
  Wire.setClock(400000);        // VL53L5CX supports up to 400 kHz

  Serial.println(F("Initializing sensor (can take ~10s)..."));
  if (!myImager.begin()) {
    Serial.println(F("ERROR: Sensor not found. Check wiring/power/I2C."));
    while (1) delay(100);
  }

  // Resolution: 8x8 (all 64 zones). Use 4*4 for 4x4.
  myImager.setResolution(8 * 8);
  imageResolution = myImager.getResolution();
  imageWidth      = sqrt(imageResolution);

  // Optional: set ranging frequency (Hz). Valid range depends on resolution/config.
  // Higher frequency = shorter timing budget = noisier measurements.
  myImager.setRangingFrequency(15); // e.g., 15 Hz

  // Optional: integration time (ms). Longer = better SNR but slower.
  // myImager.setIntegrationTime(20);

  // Optional: power/sharpener/target order, etc.
  // myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP);

  if (!myImager.startRanging()) {
    Serial.println(F("ERROR: Failed to start ranging."));
    while (1) delay(100);
  }

  Serial.print(F("Resolution: "));
  Serial.print(imageWidth);
  Serial.print("x");
  Serial.println(imageWidth);
  Serial.println(F("Columns are printed right-to-left to match SparkFun examples.\n"));
}

void loop() {
  if (myImager.isDataReady()) {
    if (myImager.getRangingData(&measurementData)) {

      // ---- 1) QUICK GRID VIEW (distance or X for invalid) ----
      // The ST library returns data transposed vs datasheet zone layout.
      // We mirror SparkFun’s example print order for visual sanity.
      Serial.println(F("Distance grid (mm), invalid -> X:"));
      for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth) {
        for (int x = imageWidth - 1; x >= 0; x--) {
          const int zone = x + y;

          // For grid view we show the first target only (t=0) if it's valid,
          // else 'X'. You can change to show min/closest valid target, etc.
          const uint8_t nb_targets = measurementData.nb_target_detected[zone];
          bool printed = false;

          for (int t = 0; t < nb_targets; t++) {
            // Per-target arrays are laid out as [target-major][zone]:
            // idx = zone + t * imageResolution
            const int idx = zone + t * imageResolution;
            if (isValidStatus(measurementData.target_status[idx])) {
              Serial.print(measurementData.distance_mm[idx]);
              printed = true;
              break; // show the first valid target for the grid
            }
          }

          if (!printed) Serial.print("X"); // or 0

          Serial.print('\t');
        }
        Serial.println();
      }
      Serial.println();

      // ---- 2) DETAILED DUMP (per-zone, per-target fields) ----
      Serial.println(F("Detailed results (per-zone / per-target):"));
      for (int zone = 0; zone < imageResolution; zone++) {
        const uint8_t nb_targets = measurementData.nb_target_detected[zone];
        const uint32_t ambient   = measurementData.ambient_per_spad[zone];
        const uint16_t spads_on  = measurementData.nb_spads_enabled[zone];

        // Zone header
        Serial.print(F("Zone "));
        Serial.print(zone);
        Serial.print(F("  ambient_per_spad="));
        Serial.print(ambient);
        Serial.print(F("  nb_spads_enabled="));
        Serial.print(spads_on);
        Serial.print(F("  nb_target_detected="));
        Serial.println(nb_targets);

        if (nb_targets == 0) {
          Serial.println(F("  (no targets)"));
          continue;
        }

        for (int t = 0; t < nb_targets; t++) {
          const int idx = zone + t * imageResolution;

          const bool valid    = isValidStatus(measurementData.target_status[idx]);
          const int16_t dist  = measurementData.distance_mm[idx];
          const uint16_t sigm = measurementData.range_sigma_mm[idx];
          const uint32_t sig  = measurementData.signal_per_spad[idx];
          const uint8_t refl  = measurementData.reflectance[idx];
          const uint8_t stat  = measurementData.target_status[idx];

          Serial.print(F("  Target "));
          Serial.print(t);
          Serial.print(F(": "));

          Serial.print(F("status="));
          Serial.print(stat);
          if (!valid) Serial.print(F(" (INVALID)"));

          Serial.print(F("  distance_mm="));
          Serial.print(dist);
          if (!valid)  Serial.print('X'); // or 0

          Serial.print(F("  range_sigma_mm="));
          Serial.print(sigm);

          Serial.print(F("  signal_per_spad="));
          Serial.print(sig);

          Serial.print(F("  reflectance="));
          Serial.println(refl);
        }
      }
      Serial.println();
    }
  }

  delay(1000); // small poll delay
}
