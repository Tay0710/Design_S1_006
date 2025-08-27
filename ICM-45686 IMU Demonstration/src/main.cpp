#include <SPI.h>
#include "ICM45686.h"

// Use VSPI bus on ESP32, CS on GPIO5
ICM456xx IMU(SPI, 5);

void setup() {
  int ret;
  Serial.begin(115200);

  // Optional: initialize SPI bus explicitly
  SPI.begin(18, 19, 23, 5); // SCK=18, MISO=19, MOSI=23, CS=5

  // --- SPI low-level WHO_AM_I test ---
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH); // CS high idle

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(5, LOW); // select IMU
  SPI.transfer(0x75 | 0x80); // 0x75 = WHO_AM_I, 0x80 = read flag
  uint8_t who_am_i = SPI.transfer(0x00); // read data
  digitalWrite(5, HIGH); // deselect
  SPI.endTransaction();
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
  inv_imu_sensor_data_t imu_data;

  // Read sensor data
  IMU.getDataFromRegisters(imu_data);

  
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