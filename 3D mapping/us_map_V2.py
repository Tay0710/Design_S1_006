"""
us_map_V2.py
--------------------------
Generates corner points (Up + Right) from ultrasonic sensors.
Corner = intersection of Up height and Right wall distance.
Pink points mark these corner locations.
"""

import numpy as np
import pandas as pd
import open3d as o3d

# === Sensor Offsets (m) ===
offsetU = np.array([0.021, 0.0, 0.132])
offsetR = np.array([0.040, -0.052, 0.035])

# === Parameters ===
TIME_TOL = 0.04   # seconds
MIN_DIST = 0.05    # ignore < 5 cm
POINT_SIZE = 6.0

# === Load rotation matrices ===
def load_rotation_matrices(rot_csv):
    rot = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=range(1, 10))
    times = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=(0,))
    return times, rot.reshape(-1, 3, 3)

# === Parse distance ===
def parse_distance(val):
    try:
        v = float(val)
        if v <= 0:
            return np.nan
        return v / 1000.0
    except:
        return np.nan

# === Rotate + translate ===
def to_world(local_vec, rot_mat, drone_pos):
    return drone_pos + rot_mat @ local_vec

# === Align Up and Right readings by time ===
def align_up_right(us_df, time_tol=TIME_TOL):
    df = us_df.copy()
    df["time"] = pd.to_numeric(df["time"], errors="coerce")
    df["type"] = df["type"].astype(str).str.upper().str.strip()
    df["dist_m"] = df["distance"].apply(parse_distance)
    df = df.dropna(subset=["time", "dist_m"])

    up_df = df[df["type"] == "U"][["time", "dist_m"]].rename(columns={"dist_m": "U"}).sort_values("time")
    right_df = df[df["type"] == "R"][["time", "dist_m"]].rename(columns={"dist_m": "R"}).sort_values("time")

    if up_df.empty or right_df.empty:
        return pd.DataFrame(columns=["time", "U", "R"])

    # Merge nearest timestamps
    aligned = pd.merge_asof(up_df, right_df, on="time", direction="nearest", tolerance=time_tol)
    return aligned.dropna()

# === Visualization ===
def visualize_open3d(points, drone_positions):
    geoms = []

    # Corner points (pink)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    pc.paint_uniform_color([1.0, 0.2, 0.8])
    geoms.append(pc)

    # Drone trajectory (red)
    traj = o3d.geometry.LineSet()
    traj.points = o3d.utility.Vector3dVector(drone_positions)
    traj.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(drone_positions) - 1)])
    traj.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(drone_positions) - 1)])
    geoms.append(traj)

    # Final marker
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    sphere.translate(drone_positions[-1])
    sphere.paint_uniform_color([1, 0, 0])
    geoms.append(sphere)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    geoms.append(axis)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Up + Right Ultrasonic Corners")
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

    aligned = align_up_right(us, TIME_TOL)
    if aligned.empty:
        print("⚠️ No aligned U/R readings found.")
        return

    corner_points = []

    for _, row in aligned.iterrows():
        t = float(row["time"])
        dU = row["U"]
        dR = row["R"]
        if np.isnan(dU) or np.isnan(dR):
            continue

        # Find nearest rotation + drone pos
        idx = np.searchsorted(traj_time, t, side="right")
        if idx >= len(traj_time):
            continue

        rot_mat = rot_mats[min(idx, len(rot_mats) - 1)]
        drone_pos = drone_positions[idx]

        # Compute local vectors
        vU = np.array([0.0, 0.0, dU]) + offsetU
        vR = np.array([0.0, -dR, 0.0]) + offsetR  # assuming right is -Y

        # Corner = combination of Up height + Right width
        corner_local = np.array([0.0, vR[1], vU[2]])
        corner_world = to_world(corner_local, rot_mat, drone_pos)
        corner_points.append(corner_world)

    if not corner_points:
        print("⚠️ No corner points generated.")
        return

    print(f"✅ Generated {len(corner_points)} up-right corner points.")
    visualize_open3d(np.array(corner_points), drone_positions)

if __name__ == "__main__":
    main()
