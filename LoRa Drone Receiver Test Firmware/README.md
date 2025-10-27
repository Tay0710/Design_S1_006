# ELEC5550 Group Project (Team 0-06)

**Students Involved:**
- Aidan	Wheller
- Dorothy Jong
- Emersyn Johnson
- Owen Spicer
- Samuel Dixon
- Taylah Karran

## 📁 Project Overview - LoRa Drone Receiver Test Firmware

This folder contains the firmware to test the range of the LoRa receiver on board the drone. It simply prints the payload and corresponding RSSI, SNR, and time between each LoRa message for each message received. 

---
## Execution Order
### **A. Install VSCode and PlatformIO**
1. To run PlatformIO, download VSCode from the following [link](https://platformio.org/platformio-ide).
1. Open Extensions in VSCode and download PlatformIO from VSCode Extensions.

### **B. Uploading the Firmware**
1. Open the LORA_DRONE_RECEIVER folder as a project in PlatformIO.
1. Connect the flight computer PCB to your PC via the micro-USB port.
1. Click the upload button on the PlatformIO to upload the firmware. Note: you may need to enter boot mode. To do this, open the case and press the boot button when resetting or powering on the board.

### **C. Testing the range of the LoRa receiver**
1. Power the LoRa Kill Switch and press the start button to begin LoRa transmission. Keep the LoRa kill switch at the base station.
1. Connect the flight computer to a Serial Monitor using a micro-USB cable. Make sure that the motors are NOT powered using a LiPo battery.
1. Walk around the designated operation zone with the drone and view the messages as well as the RSSI, SNR, and time between messages on the Serial Monitor connected to the flight computer to ensure that the LoRa antenna has sufficient range within the operation zone. 
