# ELEC5550 Group Project

**Students Involved:**
- Aidan	Wheller
- Dorothy Jong
- Emersyn Johnson
- Owen Spicer
- Samuel Dixon
- Taylah Karran

## 📁 Project Overview - Flight Controller Firmware (BetaFlight)

This folder contains the flight controller firmware intended for the STM32F405RGT6 microcontroller on the flight controller PCB. The purpose of this firmware is to manage and stabilise the vehicle’s flight dynamics, processing IMU data and control inputs to generate precise motor and actuator commands.
The firmware is based on the open-source project *BetaFlight*, with a custom configuration tailored specifically for this system. 


---
## Execution Order
### **A. Install STM32CubeProgrammer**
1. Download the STM32CubeProgrammer from this [link](https://www.st.com/en/development-tools/stm32cubeprog.html).
1. Make sure to download the correct version for your PC.

### **B. Flashing the Firmware**
1. Download FLIGHT_CONTROLLER_V1.hex.
1. Open the *STM32CubeProgrammer*.
1. Using an ST-Link, connect SWDIO, SWDCLK, VDD and GND on the flight controller PCB to a PC running STM32CubeProgrammer.
1. Select 'ST-LINK' as the flashing method.
1. Go to the 'Erase & Programming' tab and select the file path of the FLIGHT_CONTROLLER_V1.hex file.
1. Click 'Start Programming' to flash the firmware onto 

