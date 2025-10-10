#include <SPI.h>
#include "ICM45686.h"

#define AP_SDI2 36 // IMU / ICM - PSRAM
#define AP_CLK2 37 // SDO = MISO; SDI = MOSI
#define AP_SDO2 35
#define AP_CS2 38

#define BOOT_PIN 0

bool mode = false;         // toggled by BOOT button
unsigned long lastPress = 0;
const unsigned long debounceDelay = 200; // ms

SPIClass psramSPI(HSPI); // Custom: using Flash/PSRAM bus for IMU

// Use VSPI bus on ESP32, CS on GPIO5
ICM456xx IMU(psramSPI, AP_CS2);

void setup() {
  int ret;
  Serial.begin(115200);
  pinMode(BOOT_PIN, INPUT_PULLUP);
  delay(2000);
  Serial.println("check");
  // Optional: initialize SPI bus explicitly
  // SPI.begin(18, 19, 23, 5); // SCK=18, MISO=19, MOSI=23, CS=5
  psramSPI.begin(AP_CLK2, AP_SDO2, AP_SDI2, AP_CS2);

  // --- SPI low-level WHO_AM_I test ---
  pinMode(AP_CS2, OUTPUT);
  digitalWrite(AP_CS2, HIGH); // CS high idle

  psramSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(AP_CS2, LOW); // select IMU
  psramSPI.transfer(0x75 | 0x80); // 0x75 = WHO_AM_I, 0x80 = read flag
  uint8_t who_am_i = psramSPI.transfer(0x00); // read data
  digitalWrite(AP_CS2, HIGH); // deselect
  psramSPI.endTransaction();
  Serial.print("WHO_AM_I = 0x");
  Serial.println(who_am_i, HEX);
  // -----------------------------------

  // Initialize the IMU using the library
  ret = IMU.begin();
  if (ret != 0) {
    Serial.print("ICM456xx initialization failed: ");
    Serial.println(ret);
    while(1);
  }

  // Configure accelerometer and gyro (up to 8kbytes, ususal 2kbytes, FIFO rate)
  // I am assuming it is in Low Noise Mode by default. 
  IMU.startAccel(100, 2);     // 100 Hz(max 6400) , ±2/4/8/16/32 g
  IMU.startGyro(100, 15.625);    // 100 Hz, ±15.625/31.25/62.5/125/250/500/1000/2000/4000 dps

  delay(100); // wait for IMU startup
}


void loop() {
    // --- Handle BOOT button toggle ---
  if (digitalRead(BOOT_PIN) == LOW) {  // pressed (active LOW)
    if (millis() - lastPress > debounceDelay) {
      mode = !mode;   // toggle mode
      Serial.print("Mode toggled to: ");
      Serial.println(mode);
      lastPress = millis();
    }
  }
  
  
  
  inv_imu_sensor_data_t imu_data;

  // Read sensor data
  IMU.getDataFromRegisters(imu_data);

  // bool boot button pressed
  // if true, return mode = 1
  // Next time boot button pressed, return mode = 0
  if (mode == true) {
    // Print data 
    // Data is a step value from -2^15 to +2^15 representing the full scale range
    Serial.print("AccelX: "); Serial.println(imu_data.accel_data[0]);
    Serial.print("AccelY: "); Serial.println(imu_data.accel_data[1]);
    Serial.print("AccelZ: "); Serial.println(imu_data.accel_data[2]);
    Serial.print("GyroX: ");  Serial.println(imu_data.gyro_data[0]);
    Serial.print("GyroY: ");  Serial.println(imu_data.gyro_data[1]);
    Serial.print("GyroZ: ");  Serial.println(imu_data.gyro_data[2]);
    Serial.print("Temp: ");   Serial.println(imu_data.temp_data);

    delay(10); // ~100 Hz loop
  }
}