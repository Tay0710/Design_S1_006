"""
x0_position_pipeline.py
-----------------------
Coordinates the sequential execution of data-processing stages to estimate
drone motion and position for the ELEC5550 Indoor 3D Mapping  Design Project (2025).

Pipeline stages:
    1) Pixel --> angular-rate (PMW3901)
       Converts optical-flow pixel displacements (dx, dy) to angular rates (rad/s).
    2) ToF --> height (VL53L7CX)
       Processes Time-of-Flight measurements to obtain drone altitude above ground.
    3) Interpolate heights
       Aligns ToF-derived heights to optical-flow timestamps for velocity calculations.
    4) Angular-rate + height --> v_x, v_y
       Combines angular rates with altitude to compute linear velocities.
    5) IMU --> rotation matrices (AHRS)
       Runs an IMU AHRS filter to produce 3×3 body --> world rotation matrices.
    6) Body-frame → world-frame conversion
       Rotates body-frame velocities into the world frame and integrates to position.

Usage:
    Run the full pipeline on a dataset:
        $ python x0_position_pipeline.py

Outputs (CSV):
    - optical_flow_angular_rates.csv
        Angular rates (ω_x, ω_y) from optical flow.
    - ToF_heights.csv
        Altitude estimates from ToF.
    - ToF_heights_interp.csv
        ToF heights interpolated onto optical-flow timestamps.
    - xy_velocities.csv
        Planar velocities (v_x, v_y).
    - rotation_matrices.csv
        3×3 orientation matrices per timestep.
    - xy_velocities_to_world_frame.csv
        World-frame velocities and integrated positions.

Final deliverable:
    A synchronized world-frame trajectory (position and velocity) suitable for
    mapping and fusion with additional sensing.
"""

import time
import numpy as np

from x1_pixel_to_angular_rate import main as pixel_to_angular_rate
from x2_height_from_ToF import main as height_from_ToF
from x3_interpolate_heights import main as interpolate_heights
from x4_xy_velocity_calculation import main as xy_velocity_calculation
from x5_rotation_matrix import main as rotation_matrix
from x6_convert_to_world_frame import main as convert_to_world_frame

def cut_data(stage_0_input_path, stage_1_input_path, stage_2_input_path, stage_5_input_path, stage_1_input_path_cropped, stage_2_input_path_cropped, stage_5_and_7_input_path_cropped):
    """Crop three CSV logs to a [start, end] window read from a CSV."""
    data_0 = np.genfromtxt(stage_0_input_path, delimiter=",", skip_header=1)
    start_time, end_time = data_0[0], data_0[1]

    def filter_file(input_path, output_path, start, end):
        """Write only rows with start ≤ t ≤ end, preserving header/format."""
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
                kept.append(line)

        with open(output_path, "w") as f:
            f.writelines(kept)

    filter_file(stage_1_input_path, stage_1_input_path_cropped, start_time, end_time)
    filter_file(stage_2_input_path, stage_2_input_path_cropped, start_time, end_time)
    filter_file(stage_5_input_path, stage_5_and_7_input_path_cropped, start_time, end_time)

    print(f"Files updated and saved with cut data: start={start_time}, end={end_time}")

def main():
    """Run the pipeline: crop logs, then execute stages 1–6."""
    t0 = time.time()

    data_name = "demo_data/"

    # Input paths for the position pipeline stages
    stage_0_input_path = "../optical_flow_method_data/combined_samples/" + data_name + "data_times.csv"
    stage_1_input_path = "../optical_flow_method_data/combined_samples/" + data_name + "of_PMW3901.csv"
    stage_2_input_path = "../optical_flow_method_data/combined_samples/" + data_name + "tof_L7.csv"
    stage_5_input_path = "../optical_flow_method_data/combined_samples/" + data_name + "imu_ICM45686.csv"

    # Cropped input paths for the position pipeline stages
    stage_1_input_path_cropped = "../optical_flow_method_data/combined_samples/" + data_name + "download_of_cropped.csv"
    stage_2_input_path_cropped = "../optical_flow_method_data/combined_samples/" + data_name + "download_tof_cropped.csv"
    stage_5_input_path_cropped = "../optical_flow_method_data/combined_samples/" + data_name + "download_imu_cropped.csv"

    cut_data(stage_0_input_path, stage_1_input_path, stage_2_input_path, stage_5_input_path, stage_1_input_path_cropped, stage_2_input_path_cropped, stage_5_input_path_cropped)

    print("\n=== Stage 1: pixel --> angular-rate ===")
    pixel_to_angular_rate(stage_1_input_path_cropped)

    print("\n=== Stage 2: ToF --> height ===")
    height_from_ToF(stage_2_input_path)

    print("\n=== Stage 3: interpolate heights ===")
    interpolate_heights()
    
    print("\n=== Stage 4: angular-rate + height --> v_x, v_y ===")
    xy_velocity_calculation()
    
    print("\n=== Stage 5: rotation matrix from IMU orientation ===")
    rotation_matrix(stage_5_input_path_cropped)
    
    print("\n=== Stage 6: convert the velocities to world frame ===")
    convert_to_world_frame()

    print(f"\nPipeline complete in {time.time() - t0:.2f}s")

if __name__ == "__main__":
    main()