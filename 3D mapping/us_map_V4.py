"""
us_map_V3.py
------------------------------
Shows only ACTUAL ultrasonic points plus a synthetic floor (Down) point
at z = 0 for each Up timestamp. Extrapolation/interpolation is disabled.

Colours:
  • Up (U)      : purple        [0.75, 0.20, 0.90]
  • Left (L)    : blue          [0.10, 0.50, 1.00]
  • Right (R)   : orange        [1.00, 0.55, 0.10]
  • Floor (D*)  : black         [0.00, 0.00, 0.00]
  • Trajectory  : (unchanged)   [1.00, 0.176, 0.667]
"""

import numpy as np
import pandas as pd
import open3d as o3d
from scipy.spatial import cKDTree  # kept for future use, not used now

# === Sensor Offsets (m) ===
offsetU = np.array([0.021,  0.0,   0.132])
offsetD = np.array([0.019,  0.0,   0.000])  # used only for horizontal XY under the drone
offsetL = np.array([0.012,  0.080, 0.035])
offsetR = np.array([0.040, -0.052, 0.035])

# === Parameters ===
TIME_TOL = 0.055
POINT_SIZE = 5.5

# (Extrapolation parameters kept for future; not used now)
INTERP_STEPS = 12
MAX_DIST_MATCH = 1.2
BOOST_STEPS = 28
BOOST_MAX_DIST = 2.0
BOOST_K_NEIGHBORS = 2
BOOST_OVERSHOOT = 0.10

# === Load rotation matrices ===
def load_rotation_matrices(rot_csv):
    rot = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=range(1, 10))
    times = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=(0,))
    return times, rot.reshape(-1, 3, 3)

def parse_distance(v):
    try:
        val = float(v)
        if val <= 0:
            return np.nan
        return val / 100.0  # mm → m
    except:
        return np.nan

def to_world(local_vec, rot_mat, drone_pos):
    return drone_pos + rot_mat @ local_vec

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

# === NEW: create synthetic floor points (Down at z=0) for each Up timestamp ===
def create_floor(up_times, traj_time, rot_mats, drone_positions):
    """
    For each Up timestamp, generate a single 'Down' world point on the floor plane (z=0)
    located horizontally under the Down sensor:
      world_point = (drone_pos + R * offsetD) with z forced to 0
    Returns: np.ndarray (N,3)
    """
    out = []
    for t in up_times:
        idx = np.searchsorted(traj_time, t, side="right")
        if idx >= len(traj_time):
            continue
        rot_mat = rot_mats[min(idx, len(rot_mats) - 1)]
        drone_pos = drone_positions[idx]
        # Horizontal location of the Down sensor in world frame
        p = drone_pos + rot_mat @ offsetD
        out.append(np.array([p[0], p[1], 0.0]))  # clamp to floor plane
    return np.array(out) if len(out) else np.empty((0,3))

# ---------- visual ----------
def visualize_actual_only(actual_U, actual_L, actual_R, floor_D, drone_positions):
    geoms = []

    if len(actual_U):
        pc_u = o3d.geometry.PointCloud()
        pc_u.points = o3d.utility.Vector3dVector(actual_U)
        pc_u.paint_uniform_color([0.75, 0.20, 0.90])  # purple
        geoms.append(pc_u)

    if len(actual_L):
        pc_l = o3d.geometry.PointCloud()
        pc_l.points = o3d.utility.Vector3dVector(actual_L)
        pc_l.paint_uniform_color([0.10, 0.50, 1.00])  # blue
        geoms.append(pc_l)

    if len(actual_R):
        pc_r = o3d.geometry.PointCloud()
        pc_r.points = o3d.utility.Vector3dVector(actual_R)
        pc_r.paint_uniform_color([1.00, 0.55, 0.10])  # orange
        geoms.append(pc_r)

    if len(floor_D):
        pc_d = o3d.geometry.PointCloud()
        pc_d.points = o3d.utility.Vector3dVector(floor_D)
        pc_d.paint_uniform_color([0.00, 0.00, 0.00])  # black
        geoms.append(pc_d)

    # Trajectory (unchanged colour)
    traj = o3d.geometry.LineSet()
    traj.points = o3d.utility.Vector3dVector(drone_positions)
    traj.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(drone_positions) - 1)])
    traj.colors = o3d.utility.Vector3dVector([[1.0, 0.176, 0.667] for _ in range(len(drone_positions) - 1)])
    geoms.append(traj)

    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    sphere.translate(drone_positions[-1])
    sphere.paint_uniform_color([1.0, 0.176, 0.667])
    geoms.append(sphere)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    geoms.append(axis)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Ultrasonic - Actual Only + Floor (z=0)")
    for g in geoms:
        vis.add_geometry(g)
    opt = vis.get_render_option()
    opt.point_size = POINT_SIZE
    vis.run()
    vis.destroy_window()

# ---------- main ----------
def main(us_input_cropped):
    # files
    traj = pd.read_csv("../optical_flow_method_data/xy_velocities_to_world_frame.csv")
    times_mat, rot_mats = load_rotation_matrices("../optical_flow_method_data/rotation_matrices.csv")
    us = pd.read_csv(us_input_cropped)

    traj_time = traj["time (s)"].values
    drone_positions = traj[["pos_world_x", "pos_world_y", "pos_world_z"]].values

    # --- actual points per-sensor (U, L, R only) ---
    actual_U, actual_L, actual_R = [], [], []
    us["type"] = us["type"].astype(str).str.strip().str.upper()

    # Collect Up timestamps for floor synthesis
    up_times = []

    for _, row in us.iterrows():
        t = row["time"]
        d = parse_distance(row["distance"])
        s = row["type"]
        if s not in {"U", "L", "R"} or np.isnan(d):
            if s == "U":  # still track timestamps even if distance invalid? safer to require valid
                # only add if valid distance so alignment matches real U points
                pass
            continue

        idx = np.searchsorted(traj_time, t, side="right")
        if idx >= len(traj_time):
            continue
        rot_mat = rot_mats[min(idx, len(rot_mats) - 1)]
        drone_pos = drone_positions[idx]

        if s == "U":
            v = np.array([0, 0, d]) + offsetU
            up_times.append(t)
            actual_U.append(to_world(v, rot_mat, drone_pos))
        elif s == "L":
            v = np.array([d, 0, 0]) + offsetL
            actual_L.append(to_world(v, rot_mat, drone_pos))
        elif s == "R":
            v = np.array([-d, 0, 0]) + offsetR
            actual_R.append(to_world(v, rot_mat, drone_pos))

    actual_U = np.array(actual_U) if len(actual_U) else np.empty((0,3))
    actual_L = np.array(actual_L) if len(actual_L) else np.empty((0,3))
    actual_R = np.array(actual_R) if len(actual_R) else np.empty((0,3))

    # --- Synthesize floor (Down) points at z=0 for each Up timestamp ---
    floor_D = create_floor(up_times, traj_time, rot_mats, drone_positions)

    # ====== EVERYTHING BELOW (corners/interpolation/boosts) DISABLED ======
    # # Pairs / corners
    # pairs = {"UR": ("U", "R"), "UL": ("U", "L")}  # (D removed)
    # ... (commented out)

    # # Generic / boosted interpolation
    # generic_interp = np.empty((0,3))
    # left_interp = np.empty((0,3))
    # right_interp = np.empty((0,3))
    # ======================================================================

    print(f"✅ Up points:   {len(actual_U)}")
    print(f"✅ Left points: {len(actual_L)}")
    print(f"✅ Right points:{len(actual_R)}")
    print(f"✅ Floor (D*):  {len(floor_D)}")

    visualize_actual_only(actual_U, actual_L, actual_R, floor_D, drone_positions)

    # Return only the actuals + floor for now
    all_actual = np.vstack([a for a in [actual_U, actual_L, actual_R, floor_D] if len(a)]) \
                 if any(len(a) for a in [actual_U, actual_L, actual_R, floor_D]) else np.empty((0,3))
    return all_actual, actual_U, actual_L, actual_R, floor_D

if __name__ == "__main__":
    main()
