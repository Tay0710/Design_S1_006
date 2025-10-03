"""
x0_position_pipeline.py
-----------------------
Coordinates the sequential execution of all data-processing stages for 
estimating drone position and motion in the ELEC5550 Design Project (2025), 
"Indoor 3D Mapping Drone".

Pipeline stages:
    1. Pixel → Angular-rate conversion
        - Converts raw pixel displacement (dx, dy) from the PMW3901 optical 
          flow sensor into angular rates (rad/s).
    2. ToF → Height estimation
        - Processes raw VL53L7CX Time-of-Flight (ToF) measurements to obtain
          drone altitude above ground.
    3. Interpolate heights
        - Aligns ToF-derived heights with optical flow timestamps for use in
          velocity calculations.
    4. Angular-rate + Height → v_x, v_y velocity estimation
        - Combines optical flow angular rates with altitude to compute linear 
          velocities in the X and Y directions.
    5. Rotation matrices from IMU orientation
        - Runs an AHRS filter on IMU gyroscope/accelerometer data to compute
          orientation as 3×3 rotation matrices.
    6. Convert velocities to world frame
        - Rotates body-frame velocities into the world frame using the IMU 
          orientation matrices and integrates to world-frame position.
    7. IMU acceleration integration to position
        - Integrates IMU accelerations (with motion detection + detrending) 
          to provide an independent position estimate from dead reckoning.
    8. Estimator (EKF)
        - Fuses IMU and optical flow/ToF world-frame estimates using an 
          Extended Kalman Filter to obtain a final smoothed trajectory.

Usage:
    Run this script directly to execute the full pipeline:
        $ python x0_position_pipeline.py

Outputs:
    Intermediate CSVs:
        - optical_flow_angular_rates.csv : Angular rates (ωx, ωy) from optical flow.
        - ToF_heights.csv                : Altitude estimates from ToF.
        - ToF_heights_interp.csv         : ToF heights interpolated to optical flow times.
        - xy_velocities.csv              : Planar velocities (vx, vy).
        - rotation_matrices.csv          : Orientation matrices (3×3 per timestep).
        - xy_velocities_to_world_frame.csv : World-frame velocities + positions.
        - imu_position.csv               : IMU-only dead reckoning (pos, vel).
        - fused_position.csv             : Final fused estimates (pos, vel) from EKF.

    Final deliverable:
        - Fused trajectory with synchronized estimates of position and velocity
          in the world frame, suitable for mapping and integration.
"""


import time
import numpy as np

from x1_pixel_to_angular_rate import main as pixel_to_angular_rate
from x2_height_from_ToF import main as height_from_ToF
from x3_interpolate_heights import main as interpolate_heights
from x4_xy_velocity_calculation import main as xy_velocity_calculation
from x5_rotation_matrix import main as rotation_matrix
from x6_convert_to_world_frame import main as convert_to_world_frame
from x7_imu_integration_to_position import main as imu_integration_to_position
from x8_estimator import main as estimator

import numpy as np

def cut_data(stage_0_input_path, stage_1_input_path, stage_2_input_path, stage_5_and_7_input_path, stage_1_input_path_cropped, stage_2_input_path_cropped, stage_5_and_7_input_path_cropped):
    # Stage 0 file is just two numbers: start, end
    data_0 = np.genfromtxt(stage_0_input_path, delimiter=",", skip_header=1)
    start_time, end_time = data_0[0], data_0[1]

    def filter_file(input_path, output_path, start, end):
        with open(input_path, "r") as f:
            lines = f.readlines()

        header = lines[0]
        kept = [header]

        for line in lines[1:]:
            if not line.strip():
                continue
            parts = line.split(",")
            try:
                t = float(parts[0])
            except ValueError:
                continue
            if start <= t <= end:
                kept.append(line)  # keep original formatting

        with open(output_path, "w") as f:
            f.writelines(kept)

    # Apply to the three main files
    filter_file(stage_1_input_path, stage_1_input_path_cropped, start_time, end_time)
    filter_file(stage_2_input_path, stage_2_input_path_cropped, start_time, end_time)
    filter_file(stage_5_and_7_input_path, stage_5_and_7_input_path_cropped, start_time, end_time)

    print(f"Files updated and saved with cut data: start={start_time}, end={end_time}")


def main():
    t0 = time.time()

    data_name = "22_09_25_MILC/7_lyco_lab/"

    stage_0_input_path = "../optical_flow_method_data/combined_samples/" + data_name + "data_times.csv"
    stage_1_input_path = "../optical_flow_method_data/combined_samples/" + data_name + "of_PMW3901.csv"
    stage_2_input_path = "../optical_flow_method_data/combined_samples/" + data_name + "tof_L7.csv"
    stage_5_and_7_input_path = "../optical_flow_method_data/combined_samples/" + data_name + "imu_ICM45686.csv"

    stage_1_input_path_cropped = "../optical_flow_method_data/combined_samples/" + data_name + "download_of_cropped.csv"
    stage_2_input_path_cropped = "../optical_flow_method_data/combined_samples/" + data_name + "download_tof_cropped.csv"
    stage_5_and_7_input_path_cropped = "../optical_flow_method_data/combined_samples/" + data_name + "download_imu_cropped.csv"

    cut_data(stage_0_input_path, stage_1_input_path, stage_2_input_path, stage_5_and_7_input_path, stage_1_input_path_cropped, stage_2_input_path_cropped, stage_5_and_7_input_path_cropped)

    print("\n=== Stage 1: pixel → angular-rate ===")
    pixel_to_angular_rate(stage_1_input_path_cropped)

    print("\n=== Stage 2: ToF → height ===")
    height_from_ToF(stage_2_input_path)

    print("\n=== Stage 3: interpolate heights ===")
    interpolate_heights()
    
    print("\n=== Stage 4: angular-rate + height → v_x, v_y ===")
    xy_velocity_calculation()
    
    print("\n=== Stage 5: rotation matrix from IMU orientation ===")
    rotation_matrix(stage_5_and_7_input_path_cropped)
    
    print("\n=== Stage 6: convert the velocities to world frame ===")
    convert_to_world_frame()

    print(f"\nPipeline complete in {time.time() - t0:.2f}s")

if __name__ == "__main__":
    main()