# ELEC5550 Group Project (Team 0-06)

**Students Involved:**
- Aidan	Wheller
- Dorothy	Jong
- Emersyn Johnson
- Owen Spicer
- Samuel Dixon
- Taylah Karran

## 🚀 Project Summary

This project integrates hardware, firmware, and design elements to develop a fully functional autonomous 3D mapping drone system. This iteration of the design involves a drone capable of autonomously navigating the operation environment of Lv4 EECE, a LoRa-based kill switch which covers the entire operation environment, a mapping test rig used to manually collect data from 2 independent mapping systems, and a 3D mapping python pipeline which runs on the base station, used to convert raw mapping data from the test rig into 2 independent maps plus a combined map.

The next iteration of this design aims to combine the mapping test rig with the autonomous drone platform, creating a fully integrated autonomous 3D mapping drone system which follows the system architecture diagram below.

![System Architecture](https://github.com/Tay0710/Design_S1_006/blob/main/System%20Integration%20Diagrams-Hardware%20%26%20Software%202%20(2).png)


## 📁 Folder Structure

```
📁 Root Directory

├── 📂 3D Mapping
│   └── TO CONFIRM...
│
├── 📂 CAD Design
│   └── 3D-printed Parts
│   └── Full_Model
│   └── LoRa Kill Switch
│
├── 📂 Drone Firmware
│   └── Flight Computer Firmware
│   └── Flight Controller Firmware
│
├── 📂 LoRa Drone Receiver Test Firmware
│   └── LORA_DRONE_RECEIVER
│
├── 📂 LoRa Kill Switch Firmware
│   └── LORA_TRANSMITTER_BUTTON
│
├── 📂 PCB Design
│   └── ESP32_BOARD_V2
│   └── STM32_BOARD_V3
│
├── 📂 Test Rig Firmware
│   └── Test Rig Data Collection
│   └── Test Rig FreeRTOS
│
└── 📂 Videos
```

