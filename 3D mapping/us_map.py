"""
us_map.py
-----------------------

Generates a 3D point cloud from Ultrasonic sensor data,
mapped onto drone positions and IMU orientation (rotation matrices).

Overview:
    - Crops the ultrasonic file based on start and end times from data_times.csv.
    - One distance per timestamp for each direction (U, D, L, R).
    - Each reading corresponds to a fixed sensor orientation in the body frame.
    - For each ultrasonic reading, the drone’s world position and orientation (rotation matrix)
      are obtained at the nearest matching timestamp.
    - Distances are expressed as a local body-frame vector (with per-sensor offsets),
      rotated into the world frame, then translated by the drone position.
    - All points are accumulated into a single point cloud representing the mapped scene.
    - Two visualisation backends are provided:
        * Open3D (interactive 3D point cloud + trajectory)
        * Matplotlib (3D scatter plot with text labels)

Inputs:
    - xy_velocities_to_world_frame.csv
        Columns:
            time (s), v_world_x, v_world_y, v_world_z,
            pos_world_x, pos_world_y, pos_world_z
    - rotation_matrices.csv
        Columns:
            time (s), r00 … r22 (flattened 3×3 rotation matrix per row)
    - fake_ultrasonic.csv
        Columns:
            time, type, distance   (type ∈ {U, D, L, R}; distance in mm)
    - data_times.csv
        Columns:
            start, end             (used to crop ultrasonic data)

Outputs:
    - Interactive Open3D window with:
        * Colored ultrasonic points (by direction)
        * Red drone trajectory line
        * Red sphere marking final drone position
    - Matplotlib 3D plot with labeled points and trajectory
"""

import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt
import os

# === Sensor Offsets (adjust as needed) ===
offsetD = np.array([0.019, 0.0, 0.0])   # [x, y, z] (m)
offsetU = np.array([0.021, 0.0, 0.132])
offsetL = np.array([0.012, 0.080, 0.035])
offsetR = np.array([0.040, -0.052, 0.035])

# === Load rotation matrices ===
def load_rotation_matrices(rot_csv):
    rot = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=range(1, 10))
    times = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=(0,))
    return times, rot.reshape(-1, 3, 3)

# === Convert a single ultrasonic reading to body-frame vector + color ===
def build_single_ultrasonic_local(distance_m, orientation):
    """Return a local body-frame vector [x,y,z,R,G,B] for a single ultrasonic reading."""
    if distance_m is None or distance_m < 0.05:
        return None

    if orientation == "D":
        R, G, B = 32, 214, 96   # Green
        vec = np.array([0.0, 0.0, -distance_m]) + offsetD
    elif orientation == "U":
        R, G, B = 141, 205, 240 # Light blue
        vec = np.array([0.0, 0.0,  distance_m]) + offsetU
    elif orientation == "L":
        R, G, B = 175, 32, 214  # Purple
        vec = np.array([0.0,  distance_m, 0.0]) + offsetL
    elif orientation == "R":
        R, G, B = 255, 0, 0     # Red
        vec = np.array([0.0, -distance_m, 0.0]) + offsetR
    else:
        return None

    return np.array([vec[0], vec[1], vec[2], R, G, B], dtype=float)

# === Rotate point to world frame ===
def rotate_point_to_world(local_vec, rot_mat, drone_pos):
    world_vec = rot_mat @ local_vec[:3]
    return (
        drone_pos[0] + world_vec[0],
        drone_pos[1] + world_vec[1],
        drone_pos[2] + world_vec[2],
        local_vec[3],
        local_vec[4],
        local_vec[5],
    )

# === Visualisation (Open3D) ===
def visualize_open3d(points, drone_positions):
    geoms = []

    # Ultrasonic points
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points[:, :3])
    pc.colors = o3d.utility.Vector3dVector(points[:, 3:6] / 255.0)
    geoms.append(pc)

    # Drone trajectory (red line)
    traj = o3d.geometry.LineSet()
    traj.points = o3d.utility.Vector3dVector(drone_positions)
    traj.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(drone_positions) - 1)])
    traj.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(drone_positions) - 1)])
    geoms.append(traj)

    # Final drone position (red sphere)
    drone_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    drone_marker.translate(drone_positions[-1])
    drone_marker.paint_uniform_color([1, 0, 0])
    geoms.append(drone_marker)

    # Coordinate frame
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    geoms.append(axis)

    o3d.visualization.draw_geometries(geoms, window_name="Ultrasonic Mapping (with Rotation)")

# === Visualisation (Matplotlib) ===
def visualize_matplotlib(points, drone_positions):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    points = np.array(points)
    drone_positions = np.array(drone_positions)

    ax.scatter(points[:, 0], points[:, 1], points[:, 2],
               c=points[:, 3:6] / 255.0, marker="s", s=15, label="Ultrasonic points")
    ax.plot(drone_positions[:, 0], drone_positions[:, 1], drone_positions[:, 2],
            c="red", label="Drone trajectory")
    ax.scatter(drone_positions[-1, 0], drone_positions[-1, 1], drone_positions[-1, 2],
               c="red", s=100, marker="o", label="Drone (final)")
    ax.text(drone_positions[-1, 0], drone_positions[-1, 1], drone_positions[-1, 2],
            f"Drone {tuple(np.round(drone_positions[-1], 2))}", color="red")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Ultrasonic + Drone Mapping (Matplotlib)")
    ax.legend()
    plt.show()

# === Main ===
def main(us_input_cropped):
    # Folder containing ultrasonic + time files
    folder = "../optical_flow_method_data/combined_samples/26_09_25_Lv4/2_mixed_straight"

    # Load trajectory, rotation, and cropped ultrasonic data
    traj = pd.read_csv("../optical_flow_method_data/xy_velocities_to_world_frame.csv")
    times_mat, rot_mats = load_rotation_matrices("../optical_flow_method_data/rotation_matrices.csv")
    us = pd.read_csv(us_input_cropped)

    all_points = []
    drone_positions = []

    traj_time = traj["time (s)"].values
    us_time = us["time"].values

    # Build drone trajectory positions
    for i in range(len(traj)):
        drone_pos = (
            traj["pos_world_x"].iloc[i],
            traj["pos_world_y"].iloc[i],
            traj["pos_world_z"].iloc[i],
        )
        drone_positions.append(drone_pos)

    # Ultrasonic mapping loop
    for i in range(len(us)):
        us_t = us_time[i]
        us_type = str(us["type"].iloc[i]).strip().upper()
        d_raw = str(us["distance"].iloc[i]).strip().upper()

        # Skip invalid entries
        if d_raw == "X" or d_raw == "" or d_raw is None:
            continue

        try:
            d_mm = float(d_raw)
        except ValueError:
            continue

        if d_mm <= 0:
            continue

        # Convert mm → m
        d_m = d_mm / 1000.0

        # Match trajectory + rotation
        match_idx = np.searchsorted(traj_time, us_t, side="right")
        if match_idx >= len(traj):
            continue
        drone_pos = (
            traj["pos_world_x"].iloc[match_idx],
            traj["pos_world_y"].iloc[match_idx],
            traj["pos_world_z"].iloc[match_idx],
        )

        rot_idx = np.searchsorted(times_mat, us_t, side="right")
        rot_mat = rot_mats[min(rot_idx, len(rot_mats) - 1)]

        # Build local and rotate to world
        local_pt = build_single_ultrasonic_local(d_m, us_type)
        if local_pt is None:
            continue

        world_pt = rotate_point_to_world(local_pt, rot_mat, drone_pos)
        all_points.append(world_pt)

    # Visualise
    if len(all_points) == 0:
        print("⚠️ No ultrasonic points were generated, skipping visualisation.")
        return

    all_points = np.array(all_points)
    visualize_open3d(all_points, drone_positions)
    # visualize_matplotlib(all_points, drone_positions)

if __name__ == "__main__":
    main()
