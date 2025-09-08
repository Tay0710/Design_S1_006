"""
x0_position_pipeline.py
-----------
This script coordinates the sequential execution of all data-processing
stages for estimating drone position and motion.

Project context:
    - Developed for ELEC5550 Design Project (2025), "Indoor 3D Mapping Drone"

Pipeline stages:
    1. Pixel → Angular-rate conversion
        - Converts raw pixel displacement (dx, dy) from the PMW3901 optical flow 
            sensor into angular rates (rad/s).
    2. ToF → Height estimation
        - Processes raw VL53L7CX Time-of-Flight (ToF) measurements to obtain
            drone altitude above ground.
    3. Interpolate heights
        - Aligns ToF-derived heights to the optical flow sample rate using
            interpolation (required for velocity calculations).
    4. Angular-rate + Height → v_x, v_y velocity estimation
        - Combines optical flow angular rates with drone altitude to compute 
            linear velocities in the X and Y directions. 

Usage:
    Run this script directly to execute the full pipeline:
        $ python x0_position_pipeline.py

Outputs:
    - Intermediate CSVs/files are written by each stage
        - optical_flow_angular_rates.csv: Time-stamped angular rates (ωx, ωy) derived from optical flow pixels.
        - ToF_heights.csv: Raw altitude estimates from the Time-of-Flight (ToF) sensor.
        - ToF_heights_interp.csv – ToF height data interpolated to align with optical flow timestamps.
        - xy_velocities.csv – Final planar velocity estimates (vx, vy) computed from angular rates and height.
        - rotation_matrices.csv – Orientation data stored as 3×3 rotation matrices for each time step.
    - Final outputs include synchronized estimates of planar velocities ***** not inclusive of rotation_matrix step!
      (v_x, v_y) for further mapping and integration.
"""

import time

from x1_pixel_to_angular_rate import main as pixel_to_angular_rate
from x2_height_from_ToF import main as height_from_ToF
from x3_interpolate_heights import main as interpolate_heights
from x4_xy_velocity_calculation import main as xy_velocity_calculation
from x5_rotation_matrix import main as rotation_matrix
from x6_convert_to_world_frame import main as convert_to_world_frame
from x7_imu_integration_to_position import main as imu_integration_to_position
# from x8_estimator import main as estimator

def main():
    t0 = time.time()

    print("\n=== Stage 1: pixel → angular-rate ===")
    pixel_to_angular_rate()

    print("\n=== Stage 2: ToF → height ===")
    height_from_ToF()

    print("\n=== Stage 3: interpolate heights ===")
    interpolate_heights()
    
    print("\n=== Stage 4: angular-rate + height → v_x, v_y ===")
    xy_velocity_calculation()
    
    print("\n=== Stage 5: rotation matrix from IMU orientation ===")
    rotation_matrix()
    
    print("\n=== Stage 6: convert the velocities to world frame ===")
    convert_to_world_frame()
    
    print("\n=== Stage 7: IMU acceleration integration to position ===")
    imu_integration_to_position()
    
    print("\n=== Stage 8: estimator (EFK) ===")
    # estimator()

    print(f"\nPipeline complete in {time.time() - t0:.2f}s")

if __name__ == "__main__":
    main()