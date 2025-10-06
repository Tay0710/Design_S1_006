"""
x0_mapping_pipeline.py
-----------------------
Coordinates the sequential execution of all data-processing stages for
generating 3D environment maps in the ELEC5550 Design Project (2025),
"Indoor 3D Mapping Drone".

Pipeline stages:
    1. Data cropping
        - Reads the start and end times from data_times.csv.
        - Crops the raw ultrasonic dataset (fake_ultrasonic.csv) to match this
          time window, producing a synchronized version
          (download_ultrasonic_cropped.csv).
    2. ToF mapping
        - Processes cropped VL53L7CX Time-of-Flight (ToF) data to generate a
          dense 3D point cloud and optional mesh reconstruction of the scene.
        - Aligns ToF frames with the drone’s estimated trajectory to map depth
          readings into world coordinates.
    3. Ultrasonic mapping
        - Processes cropped ultrasonic data to generate a complementary
          3D point cloud using IMU-derived rotation matrices for orientation.
        - Maps each directional ultrasonic reading (U, D, L, R) into the world
          frame using the same coordinate system as the ToF data.
    4. Combined visualization
        - Each mapping stage launches an Open3D visualization with:
            * Red drone trajectory and final position marker
            * Colored point clouds (ToF: blue; Ultrasonic: multi-color by direction)
            * Coordinate frame axes for world alignment

Usage:
    Run this script directly to execute the full mapping pipeline:
        $ python x0_mapping_pipeline.py

Outputs:
    Intermediate files:
        - download_ultrasonic_cropped.csv : Cropped ultrasonic data (time-synchronized).
    Final deliverable:
        - A synchronized 3D environmental map combining ToF and Ultrasonic
          point clouds, both aligned to the drone’s world-frame trajectory.
"""

from tof_map_V2 import main as tof_map
from us_map import main as us_map

import numpy as np
import time

def cut_data(data_times, us_input_path, us_input_cropped):
    # Stage 0 file is just two numbers: start, end
    data_0 = np.genfromtxt(data_times, delimiter=",", skip_header=1)
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

    filter_file(us_input_path, us_input_cropped, start_time, end_time)

    print(f"File updated and saved with cut data: start={start_time}, end={end_time}")


def main():
    t0 = time.time()
    
    data_name = "26_09_25_Lv4/2_mixed_straight/"

    data_times = "../optical_flow_method_data/combined_samples/" + data_name + "data_times.csv"
    tof_input_cropped = "../optical_flow_method_data/combined_samples/" + data_name + "download_tof_cropped.csv"
    us_input_path = "../optical_flow_method_data/combined_samples/" + data_name + "fake_ultrasonic.csv"
    us_input_cropped = "../optical_flow_method_data/combined_samples/" + data_name + "us_cropped.csv"

    cut_data(data_times, us_input_path, us_input_cropped)
    
    print("\n=== Stage 1: Generate ToF Point Cloud ===")
    tof_map(tof_input_cropped)
    
    print("\n=== Stage 2: Generate Ultrasonic Point Cloud ===")
    us_map(us_input_cropped)
    
    print(f"\nPipeline complete in {time.time() - t0:.2f}s")

if __name__ == "__main__":
    main()