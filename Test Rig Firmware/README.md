# ELEC5550 Group Project

**Students Involved:**
- Aidan	Wheller
- Dorothy Jong
- Emersyn Johnson
- Owen Spicer
- Samuel Dixon
- Taylah Karran

## 📁 Project Overview - Test Rig Firmware

This folder contains the firmware for the mapping test rig used to simulate mapping on-board the drone and firmware prepared to integrate the mapping and autonomous flight systems. Samples were collected using a custom-made ESP32-S3 flight computer PCB and the sensors listed below:
- PMW3901 optical flow camera
- 4 * VL53L7CX time of flight sensors
- 3 * MB1030 ultrasonic sensors
- ICM-45686 inertial measurement unit
- Generic SD card module

---
## Execution Order - Test Rig Data Collection
### **A. Install VSCode and PlatformIO**
1. To run PlatformIO, download VSCode from the following [link](https://platformio.org/platformio-ide).
1. Open Extensions in VSCode and download PlatformIO from VSCode Extensions.

### **B. Uploading the Firmware**1. 
1. Open the Test Rig Data Collection folder as a project in PlatformIO.
1. Connect the ESP32-S3 flight computer board to your PC via the USB micro-B port.
1. Click the upload button on the PlatformIO to upload the firmware. Note: you may need to enter boot mode. To do this, open the case and press the boot button when resetting or powering on the board

### **C. Executing a Scan**
1. Remove USB connection to laptop/PC and plug in the 2S battery. The power PCB LED should activate.
1. Secure the test rig such that the PCB is flat and press the calibration button. The white PCB LED should activate.
1. Hold the test rig still until the white PCB LED switches off, indicating setup is complete.
1. Position the test rig at the start of the sample location and press the calibration button to begin sampling (mode = TRUE).
1. Walk the intended scan route.
1. Press the calibration button to end the sample (mode = FALSE).
1. Wait 5 seconds then remove the battery and microSD card.
1. Transfer scan files to the data folder of the Design_S1_006 GitHub repository.

### **Output Files**
CSV output files:

    imu_ICM45686.csv
        returns: time, gyro x, gyro y, gyro z, accel x, accel y, accel z
        units: s, dps, dps, dps, g, g, g

    tof_L7.csv
        returns: time, type, D0, ..., D63
        units: s, [refer below], mm, ..., mm
        type: L, R, U, D (refers to the specific ToF being called upon)

    of_PMW3901.csv
        returns: time, deltaX, deltaY, SQUAL, SHUTTER
        units: s, [unitless], ..., [unitless]

    Ultra_MB1030.csv
        returns: time, type, distance
        units: s, [refer below], cm
        type: L, R, U (refers to the specific ultrasonic sensor being called upon)

---
## Test Rig FreeRTOS

Firmware was prepared to integrate the mapping and autonomous flight systems using FreeRTOS. The Test Rig FreeRTOS folder contains a PlatformIO project that can sample mapping sensors on-board the drone with minimal impact on the navigation algorithm.

### **Priority Logic**

| Task Name         | Purpose                         | Rate/Freq   | Priority   | Core   |
| ----------------- | ------------------------------- | ---------   | ---------- | ------ |
| `ultrasonic_ctrl` | Flight control via SBUS         | 20 Hz       | 🔴 High    | Core 1 |
| `tof_mapping`     | Primary mapping (ToF)           | 15 or 60Hz  | 🟠 Med     | Core 0 |
| `ultrasonic_map`  | Secondary mapping (Ultrasonics) | 20 Hz       | 🟠 Med     | Core 0 |
| `imu_logger`      | IMU raw logging                 | 100 Hz      | 🟡 Med-Low | Core 0 |
| `optflow_logger`  | Optical Flow raw logging        | 50 - 100 Hz | 🟡 Med-Low | Core 0 |
| `lora_heartbeat`  | Alive signal check              | 1 Hz        | 🟢 Low     | Core 0 |
| `logging_mgr`     | Batch CSV writes to SD          | 5–10 Hz     | 🟢 Low     | Core 0 |