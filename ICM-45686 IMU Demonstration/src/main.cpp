#include <Wire.h>
#include "ICM45686.h"

// Define your I2C pins for ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// Instantiate IMU (use default I2C address 0 or 0x68)
ICM456xx IMU(Wire, 0);


void scanI2C() {
  Serial.println("Scanning I2C bus...");
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("No I2C devices found.");
  Serial.println();
}


void setup() {
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  scanI2C(); // <--- Run I2C scan before initializing sensor

  // Initializing the ICM456XX
  int ret = IMU.begin();
  if (ret != 0) {
    Serial.print("ICM456xx initialization failed: ");
    Serial.println(ret);
    while(1);
  }

  // Accel ODR = 100 Hz, Full Scale = 16G
  IMU.startAccel(100,16);

  // Gyro ODR = 100 Hz, Full Scale = 2000 dps
  IMU.startGyro(100,2000);

  delay(100); // wait for IMU to stabilize
}

void loop() {
  inv_imu_sensor_data_t imu_data;

  // Read data from sensor
  IMU.getDataFromRegisters(imu_data);

  // Print formatted data
  Serial.print("AccelX:"); Serial.println(imu_data.accel_data[0]);
  Serial.print("AccelY:"); Serial.println(imu_data.accel_data[1]);
  Serial.print("AccelZ:"); Serial.println(imu_data.accel_data[2]);
  Serial.print("GyroX:");  Serial.println(imu_data.gyro_data[0]);
  Serial.print("GyroY:");  Serial.println(imu_data.gyro_data[1]);
  Serial.print("GyroZ:");  Serial.println(imu_data.gyro_data[2]);
  Serial.print("Temperature:"); Serial.println(imu_data.temp_data);

  delay(10); // ~100 Hz
}
