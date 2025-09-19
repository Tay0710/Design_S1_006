"""
tof_mapping_pipeline.py
-----------------------

Generates a 3D point cloud and mesh of the environment from Time-of-Flight (ToF) sensor data,
mapped onto drone positions derived from integrated velocity estimates.

Overview:
    - The ToF sensor outputs distances for a 4×4 grid of zones (D0–D15) at each timestep.
    - Each zone corresponds to a fixed angular offset in the x–y plane (±30°, ±10°).
    - For each ToF frame, the drone’s world position is obtained from the trajectory file
      (xy_velocities_to_world_frame.csv) at the nearest matching timestamp.
    - Distances are projected into world-frame coordinates using the drone position
      and the pre-defined angular offsets.
    - All ToF points are accumulated into a single point cloud representing the mapped scene.
    - A Poisson mesh is reconstructed from the point cloud for surface approximation.
    - Two visualisation backends are provided:
        * Open3D (interactive 3D point cloud + mesh + trajectory)
        * Matplotlib (3D scatter plot with text labels)

Notes:
    - The ToF and trajectory CSVs are not timestamp-aligned. Each ToF frame is matched to the
      first trajectory timestamp that occurs strictly *after* the ToF timestamp.
    - Distances are expected in millimetres in the ToF CSV and are converted to metres.

Inputs:
    - xy_velocities_to_world_frame.csv
        Columns: 
            time (s), v_world_x, v_world_y, v_world_z,
            pos_world_x, pos_world_y, pos_world_z

    - download_tof_cropped.csv
        Columns: 
            time, D0 … D15
            (Distances are in mm, invalid entries are "X")

Outputs:
    - Interactive Open3D window with:
        * Blue ToF points
        * Grey mesh reconstruction
        * Red drone trajectory line
        * Red sphere marking final drone position
    - Matplotlib 3D plot with labeled points and trajectory
"""


import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt

def tof_point_arctan(d, theta_x_deg, theta_y_deg, drone_pos=(0,0,0)):
    """Convert one ToF cell to world coords using arctan method."""
    if d is None:
        return None

    tx = np.tan(np.deg2rad(theta_x_deg))
    ty = np.tan(np.deg2rad(theta_y_deg))

    # resultant angle
    theta_r = np.arctan(np.sqrt(tx**2 + ty**2))

    if tx == 0 and ty == 0:
        x_local, y_local = 0.0, 0.0
    else:
        r_xy = d * np.sin(theta_r)
        denom = np.sqrt(tx**2 + ty**2)
        x_local = r_xy * (tx / denom)
        y_local = r_xy * (ty / denom)

    z_local = d * np.cos(theta_r)

    return (
        drone_pos[0] + x_local,
        drone_pos[1] + y_local,
        drone_pos[2] - z_local
    )

def build_points(distances, drone_pos):
    """Convert one ToF row into 16 3D points."""
    cell_angles = {
        3:  (-30, -30),
        2:  (-10, -30),
        1:  (10,  -30),
        0:  (30,  -30),

        7:  (-30, -10),
        6:  (-10, -10),
        5:  (10,  -10),
        4:  (30,  -10),

        11: (-30,  10),
        10: (-10,  10),
        9:  (10,   10),
        8:  (30,   10),

        15: (-30,  30),
        14: (-10,  30),
        13: (10,   30),
        12: (30,   30),
    }

    points = []
    for idx, (tx, ty) in cell_angles.items():
        d = distances[idx]
        pt = tof_point_arctan(d, tx, ty, drone_pos)
        if pt:
            points.append(pt)

    return points

def visualize_open3d(points, drone_positions):
    geoms = []

    # ToF points
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    pc.paint_uniform_color([0, 0, 1])
    geoms.append(pc)

    # Drone trajectory as red line
    traj = o3d.geometry.LineSet()
    traj.points = o3d.utility.Vector3dVector(drone_positions)
    traj.lines = o3d.utility.Vector2iVector([[i, i+1] for i in range(len(drone_positions)-1)])
    traj.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(drone_positions)-1)])
    geoms.append(traj)

    # Final drone position as red sphere
    drone_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
    drone_marker.translate(drone_positions[-1])
    drone_marker.paint_uniform_color([1, 0, 0])
    geoms.append(drone_marker)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    geoms.append(axis)

    o3d.visualization.draw_geometries(geoms, window_name="Full ToF Mapping")

def visualize_matplotlib(points, drone_positions):
    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection="3d")

    points = np.array(points)
    drone_positions = np.array(drone_positions)

    # Plot ToF points
    ax.scatter(points[:,0], points[:,1], points[:,2], c="blue", marker="s", s=10, label="ToF points")

    # Drone trajectory
    ax.plot(drone_positions[:,0], drone_positions[:,1], drone_positions[:,2], c="red", label="Drone trajectory")

    # Final drone position
    ax.scatter(drone_positions[-1,0], drone_positions[-1,1], drone_positions[-1,2],
               c="red", s=100, marker="o", label="Drone (final)")
    ax.text(drone_positions[-1,0], drone_positions[-1,1], drone_positions[-1,2],
            f"Drone {tuple(drone_positions[-1])}", color="red")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("ToF + Drone Mapping (Matplotlib)")
    ax.legend()
    plt.show()

def main():
    # Load trajectory + ToF data
    traj = pd.read_csv("../optical_flow_method_data/xy_velocities_to_world_frame.csv")
    tof = pd.read_csv("../optical_flow_method_data/combined_samples/17_09_25_MILC/6_lyco_lab_2/download_tof_cropped.csv")  # <-- adjust filename

    all_points = []
    drone_positions = []

    traj_time = traj["time (s)"].values
    tof_time = tof["time"].values

    # Convert drone trajectory positions
    for i in range(len(traj)):
        drone_pos = (
            traj["pos_world_x"].iloc[i],
            traj["pos_world_y"].iloc[i],
            traj["pos_world_z"].iloc[i],
        )
        drone_positions.append(drone_pos)

    # For each ToF frame, find the next trajectory timestamp after it
    for i in range(len(tof)):
        tof_t = tof_time[i]
        match_idx = np.searchsorted(traj_time, tof_t, side="right")  # first traj time > tof time
        if match_idx >= len(traj):
            continue

        drone_pos = (
            traj["pos_world_x"].iloc[match_idx],
            traj["pos_world_y"].iloc[match_idx],
            traj["pos_world_z"].iloc[match_idx],
        )

        distances = [None if str(d)=="X" else float(d)/1000.0 for d in tof.iloc[i,1:17]]
        pts = build_points(distances, drone_pos)
        all_points.extend(pts)

    # Visualise
    visualize_open3d(np.array(all_points), drone_positions)
    visualize_matplotlib(all_points, drone_positions)

if __name__ == "__main__":
    main()
