# ICM-20948 Test Data Generation

## ESP32-S3 Setup Up

### Wire Connections
- 5V or 3.3V
- GND
- SDA PIN: GPIO 21
- SCL PIN: GPIO 44
- ADO PIN: 3.3V

### Firmware Libraries Required
- Adafruit MPU6050
- Adafruit Sensor
- Wire
- AsyncTCP
- Wifi.h
- stdio.h (standard C libary)
- Ticker

### General Algorithm
1. Calibrate MPU6050
1. Setup ESP32 as Wifi
1. Connects to TCP server. Note: the rest of the code will not run unless the ESP32 is connected to a TCP server (i.e. PC)
1. Ensure AD0 pin is high
1. Once connected to a TCP server, the ESP32 sends data to the server every 0.1 seconds in the format: 
    (time, gyro.x, gyro.y, gyro.z, acceleration.x, acceleration.y, acceleration.z), including offsets from calibration


## TCP Server Setup on PC
1. Setup python venv and the required python libraries (see requirements.txt). Note: see README.md in TCP_Connection folder for exact commands required
1. Connect to ESP_TEST wifi on PC/Laptop
1. Go to Control Panel then Network and Sharing Center. Change IPv4 address from DHCP to Static IP and set it to 192.168.4.2 with subnet mask of 255.255.255.0. The rest of the settings can be left blank.
1. Run tcp_server.py
1. Check terminal to determine when the ESP32 client is connected to the server and when data is being received
1. All data received in saved into a csv in MPU6050_DATA folder

Note: Ignore the matplotlib plots being displayed

If the ESP32 is not connecting to the server, try resetting the ESP32

 
