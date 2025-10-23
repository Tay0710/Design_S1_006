# ELEC5550 Group Project

**Students Involved:**
- Aidan	Wheller
- Dorothy Jong
- Emersyn Johnson
- Owen Spicer
- Samuel Dixon
- Taylah Karran

## 📁 Project Overview - LoRa Kill Switch Firmware

This folder contains the firmware for the LoRa Kill Switch. It is intended for a LilyGo T3 S3 SX1262 board with an IPEX-connected 915 MHz T-antenna. This firmware is responsible for sending LoRa messages periodically to the LoRa receiver on the flight computer PCB. 

---
## Execution Order
### **A. Install VSCode and PlatformIO**
1. To run PlatformIO, download VSCode from the following [link](https://platformio.org/platformio-ide).
1. Open Extensions in VSCode and download PlatformIO from VSCode Extensions.

### **B. Uploading the Firmware**
1. Open the LORA_TRANSMITTER_BUTTON folder as a project in PlatformIO.
1. Connect the LilyGo T3 S3 SX1262 board to your PC via the USB-C port. Note: Ensure that the LilyGo T3 SX1262 board power switch is turned on.
1. Click the upload button on the PlatformIO to upload the firmware. Note: you may need to enter boot mode. To do this, open the case and press the boot button when resetting or powering on the board.

### **C. Using the LoRa Kill Switch**
1. To use the LoRa Kill Switch, power it using a USB-C cable.
1. Make sure that the T-antenna is connected properly.
1. Turn on the kill switch via the power switch at the bottom.
1. To start autonomous operation on the drone, press the red button once (as stated on the OLED display).
1. To stop the drone, press the red button a second time.

Note: make sure that the kill switch is not transmitting any LoRa messages when the drone is first powered on.