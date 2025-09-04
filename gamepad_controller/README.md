# Setup Instruction for Bluepad32

Note: this library is only available on ArduinoIDE (not PlatformIO). Additional setup may be required for it to work on PlatformIO

## On Arduino IDE

https://bluepad32.readthedocs.io/en/latest/plat_arduino/

1. Add ESP32 and Bluepad32 board packages to the board manager
- Official ESP32 package: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
- "Bluepad32 + ESP32" package: https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json- 

1. Install both packages through board manager

1. Use an ESP32 Dev Kit (not S3) as the controller is not compatible with BLE.

1. Select the ESP32 Dev Module (esp32_bluepad32) and upload the code through Arduino IDE

1. Once uploaded, enter pairing mode on the controller and it should pair automatically.
