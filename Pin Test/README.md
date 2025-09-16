Combined data of the ESP32 Time stamp + ICM45686 + VL53L7CX. 
The data is saved in a list. 
Then pushed over wifi to a serial monitor. 


Current COmponents in code:
SD Card module 
1x ToF L7
1x ICM45686 

Method:
Two separate SPI connections (VSPI and HSPI) (HSPI-SD Card module; VSPI-ICM)
One I2C connection (Tof L7)

Notes

Data is CSV file comes in the following order:
timestamp (in micros), distance0 (mm), distance1 (mm), ..., distance15 (mm), accelX, accelY, accelZ, gyroX, gyroY, gyroZ, 

Assuming Temperature is constant indoors, and set at UWA standard of IDK (24C?)

Units
L7: mm
accel: g's
gyro: dps

L7 Zones: 
 i=0   i=1   i=2   ... i=7
 i=8   i=9   i=10  ... i=15
 i=16  i=17  i=18  ... i=23
 ...
 i=56  i=57  i=58  ... i=63




Old IMU Sampling code; with old/ default calibration values

  float ax = imu_data.accel_data[0]*16.0/32768.0 + 0.015272462;
  float ay = imu_data.accel_data[1]*16.0/32768.0 - 0.009307082;
  float az = imu_data.accel_data[2]*16.0/32768.0 - 0.006992415;
  float gx = imu_data.gyro_data[0]*2000.0/32768.0 - 0.754845671;
  float gy = imu_data.gyro_data[1]*2000.0/32768.0 + 0.746207889;
  float gz = imu_data.gyro_data[2]*2000.0/32768.0 + 0.116757765;