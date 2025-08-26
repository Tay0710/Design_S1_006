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
timestamp (in micros), distance0 (mm), distance1 (mm), ..., distance15 (mm), accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp (C)

Units
accel: g's
gyro: 