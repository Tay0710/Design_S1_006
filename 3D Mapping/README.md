# ELEC5550 Group Project

**Students Involved:**
- Aidan	Wheller
- Dorothy Jong
- Emersyn Johnson
- Owen Spicer
- Samuel Dixon
- Taylah Karran

## 📁 Project Overview
This repository contains all scripts used to process sensor data and generate a **3D map** of an indoor environment using a drone platform equipped with:
- PMW3901 Optical Flow Sensor  
- VL53L7CX Time-of-Flight (ToF) Sensor  
- Ultrasonic Distance Sensors (U, D, L, R)  
- ICM-45686 IMU  

The system estimates **drone position and orientation** in the world frame and then maps the environment using synchronized ToF and Ultrasonic readings.

---

## Execution Order

### **A. Install the Virtual Environment**
1. From the project root folder, run:
        python -m venv venv
2. Run the virtual environment:
        .\venv\Scripts\activate
3. Install all required packages:
        pip install -r requirements.txt
Note: to get open3d working must use python 11 or below.

### **B. Position and Trajectory Estimation**
1. Open the optical flow position folder:
        cd optical_flow_method
2. Update the filepath in x0_position_pipeline.py
3. Run the position pipeline:
        python x0_position_pipeline.py

### **C. Point Cloud Generation**
1. Exit the optical flow folder:
        cd ..
2. Open the 3D mapping folder:
        cd 3D mapping
3. Update the filepath in x0_mapping_pipeline_V3.py
4. Run the mapping pipeline:
        python x0_mapping_pipeline_V3.py
##
