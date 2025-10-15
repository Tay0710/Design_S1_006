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
        - Merges ToF and Ultrasonic points into a single Open3D display:
            * Blue: ToF map
            * Multi-color: Ultrasonic directions (U, D, L, R)
            * Red: Drone trajectory and final position
            * Axes: World coordinate reference

Usage:
    Run this script directly to execute the full mapping pipeline:
        $ python x0_mapping_pipeline.py

Outputs:
    Intermediate files:
        - us_cropped.csv : Cropped ultrasonic data (time-synchronized).
    Final deliverable:
        - A synchronized 3D environmental map combining ToF and Ultrasonic
          point clouds, both aligned to the drone’s world-frame trajectory.
"""

import numpy as np
import pandas as pd
import open3d as o3d
import time
import os

from tof_map_V2 import main as tof_map
from us_map_V3 import main as us_map

# === Data Cropping Function ===
def cut_data(data_times, us_input_path, us_input_cropped):
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
                kept.append(line)

        with open(output_path, "w") as f:
            f.writelines(kept)

    filter_file(us_input_path, us_input_cropped, start_time, end_time)
    print(f"File updated and saved with cut data: start={start_time}, end={end_time}")

def visualize_combined_map(tof_points, us_interp_points, us_actual_points, us_corner_points, traj_positions):
    geoms = []

    # === ToF points (blue) ===
    if tof_points is not None and len(tof_points) > 0:
        pc_tof = o3d.geometry.PointCloud()
        pc_tof.points = o3d.utility.Vector3dVector(tof_points[:, :3])
        pc_tof.paint_uniform_color([0, 0, 1])
        geoms.append(pc_tof)

    # === Ultrasonic interpolated (orange) ===
    if us_interp_points is not None and len(us_interp_points) > 0:
        pc_us_interp = o3d.geometry.PointCloud()
        pc_us_interp.points = o3d.utility.Vector3dVector(us_interp_points[:, :3])
        pc_us_interp.paint_uniform_color([1, 0.5, 0])  # orange
        geoms.append(pc_us_interp)

    # === Ultrasonic actual (pink) ===
    if us_actual_points is not None and len(us_actual_points) > 0:
        pc_us_actual = o3d.geometry.PointCloud()
        pc_us_actual.points = o3d.utility.Vector3dVector(us_actual_points[:, :3])
        pc_us_actual.paint_uniform_color([1, 0.3, 0.8])  # pink
        geoms.append(pc_us_actual)

    # === Ultrasonic corners (cyan) ===
    if us_corner_points is not None and len(us_corner_points) > 0:
        pc_us_corners = o3d.geometry.PointCloud()
        pc_us_corners.points = o3d.utility.Vector3dVector(us_corner_points[:, :3])
        pc_us_corners.paint_uniform_color([0.3, 1, 1])  # light cyan
        geoms.append(pc_us_corners)

    # === Drone trajectory ===
    if traj_positions is not None and len(traj_positions) > 1:
        traj = o3d.geometry.LineSet()
        traj.points = o3d.utility.Vector3dVector(traj_positions)
        traj.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(traj_positions) - 1)])
        traj.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(traj_positions) - 1)])
        geoms.append(traj)

        marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        marker.translate(traj_positions[-1])
        marker.paint_uniform_color([1, 0, 0])
        geoms.append(marker)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    geoms.append(axis)

    o3d.visualization.draw_geometries(geoms, window_name="Combined ToF + Ultrasonic Map")

# === Main Execution ===
def main():
    t0 = time.time()

    data_name = "22_09_25_MILC/7_lyco_lab/"
    base_path = "../optical_flow_method_data/combined_samples/" + data_name

    data_times = base_path + "data_times.csv"
    tof_input_cropped = base_path + "download_tof_cropped.csv"
    us_input_path = base_path + "fake_ultrasonic.csv"
    us_input_cropped = base_path + "us_cropped.csv"

    # 1️⃣ Crop ultrasonic data
    cut_data(data_times, us_input_path, us_input_cropped)

    # 2️⃣ Generate ToF map
    print("\n=== Stage 1: Generate ToF Point Cloud ===")
    tof_points, traj_positions = tof_map(tof_input_cropped)

    # 3️⃣ Generate Ultrasonic map
    print("\n=== Stage 2: Generate Ultrasonic Point Cloud ===")
    us_interp_points, us_actual_points, us_corner_points = us_map(us_input_cropped)

    # 4️⃣ Combined visualization
    visualize_combined_map(tof_points, us_interp_points, us_actual_points, us_corner_points, traj_positions)

    print(f"\nPipeline complete in {time.time() - t0:.2f}s")


if __name__ == "__main__":
    main()
