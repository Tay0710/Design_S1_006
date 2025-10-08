"""
us_map_actual_and_extrapolated.py
---------------------------------
Displays:
    • Actual ultrasonic points (pink)
    • Extrapolated corner points (blue)
    • Drone path (red line)
    • Final position (red sphere)
"""

import numpy as np
import pandas as pd
import open3d as o3d

# === Sensor Offsets (m) ===
offsetU = np.array([0.021, 0.0, 0.132])
offsetD = np.array([0.019, 0.0, 0.0])
offsetL = np.array([0.012, 0.080, 0.035])
offsetR = np.array([0.040, -0.052, 0.035])

# === Parameters ===
TIME_TOL = 0.055
POINT_SIZE = 5.5

# === Load rotation matrices ===
def load_rotation_matrices(rot_csv):
    rot = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=range(1, 10))
    times = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=(0,))
    return times, rot.reshape(-1, 3, 3)

# === Parse distance safely ===
def parse_distance(v):
    try:
        val = float(v)
        if val <= 0:
            return np.nan
        return val / 1000.0
    except:
        return np.nan

# === Rotate + translate ===
def to_world(local_vec, rot_mat, drone_pos):
    return drone_pos + rot_mat @ local_vec

# === Align two sensors by nearest timestamp ===
def align_pair(df, t1, t2, tol=TIME_TOL):
    sub1 = df[df["type"] == t1][["time", "distance"]].copy()
    sub2 = df[df["type"] == t2][["time", "distance"]].copy()
    sub1["dist_m"] = sub1["distance"].apply(parse_distance)
    sub2["dist_m"] = sub2["distance"].apply(parse_distance)
    sub1 = sub1.dropna(subset=["time", "dist_m"]).sort_values("time").rename(columns={"dist_m": t1})
    sub2 = sub2.dropna(subset=["time", "dist_m"]).sort_values("time").rename(columns={"dist_m": t2})
    if sub1.empty or sub2.empty:
        return pd.DataFrame(columns=["time", t1, t2])
    merged = pd.merge_asof(sub1, sub2, on="time", direction="nearest", tolerance=tol)
    return merged.dropna()

# === Visualisation ===
def visualize_combined(actual_points, corner_points, drone_positions):
    geoms = []

    # Actual ultrasonic points (pink)
    if len(actual_points) > 0:
        pc_actual = o3d.geometry.PointCloud()
        pc_actual.points = o3d.utility.Vector3dVector(np.array(actual_points))
        pc_actual.paint_uniform_color([1.0, 0.3, 0.8])  # pink
        geoms.append(pc_actual)

    # Extrapolated corner points (blue)
    if len(corner_points) > 0:
        pc_corners = o3d.geometry.PointCloud()
        pc_corners.points = o3d.utility.Vector3dVector(np.array(corner_points))
        pc_corners.paint_uniform_color([0.2, 0.5, 1.0])  # blue
        geoms.append(pc_corners)

    # Drone trajectory (red line)
    traj = o3d.geometry.LineSet()
    traj.points = o3d.utility.Vector3dVector(drone_positions)
    traj.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(drone_positions) - 1)])
    traj.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(drone_positions) - 1)])
    geoms.append(traj)

    # Final position (red sphere)
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    sphere.translate(drone_positions[-1])
    sphere.paint_uniform_color([1, 0, 0])
    geoms.append(sphere)

    # Coordinate frame
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    geoms.append(axis)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Actual (Pink) + Extrapolated (Blue) Ultrasonic Map")
    for g in geoms:
        vis.add_geometry(g)
    opt = vis.get_render_option()
    opt.point_size = POINT_SIZE
    vis.run()
    vis.destroy_window()

# === Main ===
def main():
    traj = pd.read_csv("../optical_flow_method_data/xy_velocities_to_world_frame.csv")
    times_mat, rot_mats = load_rotation_matrices("../optical_flow_method_data/rotation_matrices.csv")
    us = pd.read_csv("../optical_flow_method_data/combined_samples/26_09_25_Lv4/2_mixed_straight/fake_ultrasonic_cropped.csv")

    traj_time = traj["time (s)"].values
    drone_positions = traj[["pos_world_x", "pos_world_y", "pos_world_z"]].values

    # === Prepare all ultrasonic points (actual pink) ===
    all_actual_points = []
    us["type"] = us["type"].astype(str).str.strip().str.upper()

    for _, row in us.iterrows():
        t = row["time"]
        d = parse_distance(row["distance"])
        s = row["type"]
        if np.isnan(d):
            continue
        idx = np.searchsorted(traj_time, t, side="right")
        if idx >= len(traj_time):
            continue
        rot_mat = rot_mats[min(idx, len(rot_mats) - 1)]
        drone_pos = drone_positions[idx]

        # Local direction
        if s == "U": v = np.array([0, 0, d]) + offsetU
        elif s == "D": v = np.array([0, 0, -d]) + offsetD
        elif s == "L": v = np.array([0, d, 0]) + offsetL
        elif s == "R": v = np.array([0, -d, 0]) + offsetR
        else:
            continue

        world_point = to_world(v, rot_mat, drone_pos)
        all_actual_points.append(world_point)

    # === Compute extrapolated corners (blue) ===
    pairs = {"UR": ("U", "R"), "UL": ("U", "L"), "DR": ("D", "R"), "DL": ("D", "L")}
    all_corners = []

    for key, (a, b) in pairs.items():
        aligned = align_pair(us, a, b, TIME_TOL)
        if aligned.empty:
            continue

        for _, row in aligned.iterrows():
            t = row["time"]
            dA, dB = row[a], row[b]
            if np.isnan(dA) or np.isnan(dB):
                continue
            idx = np.searchsorted(traj_time, t, side="right")
            if idx >= len(traj_time):
                continue
            rot_mat = rot_mats[min(idx, len(rot_mats) - 1)]
            drone_pos = drone_positions[idx]

            # Define local vectors
            if a == "U": vA = np.array([0, 0, dA]) + offsetU
            elif a == "D": vA = np.array([0, 0, -dA]) + offsetD
            if b == "R": vB = np.array([0, -dB, 0]) + offsetR
            elif b == "L": vB = np.array([0, dB, 0]) + offsetL

            # Corner local: combine Z from U/D, Y from L/R
            corner_local = np.array([0.0, vB[1], vA[2]])
            corner_world = to_world(corner_local, rot_mat, drone_pos)
            all_corners.append(corner_world)

    print(f"✅ Generated {len(all_actual_points)} actual pink points")
    print(f"✅ Generated {len(all_corners)} extrapolated blue points")

    visualize_combined(all_actual_points, all_corners, drone_positions)

if __name__ == "__main__":
    main()
