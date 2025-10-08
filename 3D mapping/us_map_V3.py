"""
us_map_wall_fill_full_boost.py
------------------------------
Extends wall fill interpolation for both sides.

Pink  : Actual ultrasonic points (U/D/L/R)
Blue  : Extrapolated corners (UR, UL, DR, DL)
LtBlue: Interpolated fills (with boosted L/R sides)
Red   : Drone path
Red o : Final position
"""

import numpy as np
import pandas as pd
import open3d as o3d
from scipy.spatial import cKDTree

# === Sensor Offsets (m) ===
offsetU = np.array([0.021, 0.0, 0.132])
offsetD = np.array([0.019, 0.0, 0.0])
offsetL = np.array([0.012, 0.080, 0.035])
offsetR = np.array([0.040, -0.052, 0.035])

# === Parameters ===
TIME_TOL = 0.055
POINT_SIZE = 5.5

# generic interp
INTERP_STEPS = 12
MAX_DIST_MATCH = 1.2

# boosted wall fills (left and right)
BOOST_STEPS = 28
BOOST_MAX_DIST = 3.0
BOOST_K_NEIGHBORS = 3
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
        return val / 1000.0
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

# ---------- interpolation helpers ----------
def interp_pairs(points_a, points_b, steps, overshoot=0.0):
    """Dense linear interpolation from A to nearest B with optional overshoot."""
    if len(points_a) == 0 or len(points_b) == 0:
        return np.empty((0,3))
    tree = cKDTree(points_b)
    out = []
    for p in points_a:
        dist, idx = tree.query(p)
        if not np.isfinite(dist):
            continue
        c = points_b[idx]
        for alpha in np.linspace(0, 1+overshoot, steps):
            out.append((1-alpha)*p + alpha*c)
    return np.array(out)

def interp_pairs_k(points_a, points_b, steps, max_dist, k=3, overshoot=0.0):
    """Link each A to up to k nearest Bs within max_dist."""
    if len(points_a) == 0 or len(points_b) == 0:
        return np.empty((0,3))
    tree = cKDTree(points_b)
    out = []
    for p in points_a:
        dists, idxs = tree.query(p, k=k, distance_upper_bound=max_dist)
        if np.isscalar(dists):
            dists = np.array([dists]); idxs = np.array([idxs])
        mask = np.isfinite(dists)
        if not np.any(mask):
            continue
        for d, idx in zip(dists[mask], idxs[mask]):
            c = points_b[idx]
            for alpha in np.linspace(0, 1+overshoot, steps):
                out.append((1-alpha)*p + alpha*c)
    return np.array(out)

# ---------- visual ----------
def visualize_combined(actual_points, corner_points, interp_points, drone_positions):
    geoms = []

    if len(actual_points) > 0:
        pc_actual = o3d.geometry.PointCloud()
        pc_actual.points = o3d.utility.Vector3dVector(np.array(actual_points))
        pc_actual.paint_uniform_color([1.0, 0.3, 0.8])  # pink
        geoms.append(pc_actual)

    if len(corner_points) > 0:
        pc_corners = o3d.geometry.PointCloud()
        pc_corners.points = o3d.utility.Vector3dVector(np.array(corner_points))
        pc_corners.paint_uniform_color([0.2, 0.5, 1.0])  # dark blue
        geoms.append(pc_corners)

    if len(interp_points) > 0:
        pc_interp = o3d.geometry.PointCloud()
        pc_interp.points = o3d.utility.Vector3dVector(np.array(interp_points))
        pc_interp.paint_uniform_color([0.6, 0.8, 1.0])  # light blue
        geoms.append(pc_interp)

    traj = o3d.geometry.LineSet()
    traj.points = o3d.utility.Vector3dVector(drone_positions)
    traj.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(drone_positions) - 1)])
    traj.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(drone_positions) - 1)])
    geoms.append(traj)

    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    sphere.translate(drone_positions[-1])
    sphere.paint_uniform_color([1, 0, 0])
    geoms.append(sphere)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    geoms.append(axis)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Ultrasonic Wall Fill (Full Left + Right Boost)")
    for g in geoms:
        vis.add_geometry(g)
    opt = vis.get_render_option()
    opt.point_size = POINT_SIZE
    vis.run()
    vis.destroy_window()

# ---------- main ----------
def main():
    # files
    traj = pd.read_csv("../optical_flow_method_data/xy_velocities_to_world_frame.csv")
    times_mat, rot_mats = load_rotation_matrices("../optical_flow_method_data/rotation_matrices.csv")
    us = pd.read_csv("../optical_flow_method_data/combined_samples/26_09_25_Lv4/2_mixed_straight/fake_ultrasonic_cropped.csv")

    traj_time = traj["time (s)"].values
    drone_positions = traj[["pos_world_x", "pos_world_y", "pos_world_z"]].values

    # --- actual points per-sensor ---
    actual = {"U": [], "D": [], "L": [], "R": []}
    us["type"] = us["type"].astype(str).str.strip().str.upper()

    for _, row in us.iterrows():
        t = row["time"]
        d = parse_distance(row["distance"])
        s = row["type"]
        if s not in actual or np.isnan(d):
            continue
        idx = np.searchsorted(traj_time, t, side="right")
        if idx >= len(traj_time):
            continue
        rot_mat = rot_mats[min(idx, len(rot_mats) - 1)]
        drone_pos = drone_positions[idx]
        if s == "U": v = np.array([0, 0, d]) + offsetU
        elif s == "D": v = np.array([0, 0, -d]) + offsetD
        elif s == "L": v = np.array([0, d, 0]) + offsetL
        elif s == "R": v = np.array([0, -d, 0]) + offsetR
        actual[s].append(to_world(v, rot_mat, drone_pos))

    for k in actual:
        actual[k] = np.array(actual[k]) if len(actual[k]) else np.empty((0,3))
    all_actual_points = np.vstack([a for a in actual.values() if len(a)]) if any(len(a) for a in actual.values()) else np.empty((0,3))

    # --- corners ---
    pairs = {"UR": ("U", "R"), "UL": ("U", "L"), "DR": ("D", "R"), "DL": ("D", "L")}
    corners = {k: [] for k in pairs}

    for key, (a, b) in pairs.items():
        aligned = align_pair(us, a, b, TIME_TOL)
        if aligned.empty:
            continue
        for _, r in aligned.iterrows():
            t = r["time"]; dA, dB = r[a], r[b]
            if np.isnan(dA) or np.isnan(dB):
                continue
            idx = np.searchsorted(traj_time, t, side="right")
            if idx >= len(traj_time):
                continue
            rot_mat = rot_mats[min(idx, len(rot_mats) - 1)]
            drone_pos = drone_positions[idx]
            if a == "U": vA = np.array([0, 0, dA]) + offsetU
            elif a == "D": vA = np.array([0, 0, -dA]) + offsetD
            if b == "R": vB = np.array([0, -dB, 0]) + offsetR
            elif b == "L": vB = np.array([0, dB, 0]) + offsetL
            corner_local = np.array([0.0, vB[1], vA[2]])
            corners[key].append(to_world(corner_local, rot_mat, drone_pos))

    for k in corners:
        corners[k] = np.array(corners[k]) if len(corners[k]) else np.empty((0,3))
    all_corners = np.vstack([a for a in corners.values() if len(a)]) if any(len(a) for a in corners.values()) else np.empty((0,3))

    # --- generic interp ---
    generic_interp = np.empty((0,3))
    if len(all_actual_points) and len(all_corners):
        generic_interp = interp_pairs(all_actual_points, all_corners, steps=INTERP_STEPS)

    # --- BOOST LEFT (existing) ---
    left_blocks = []
    if len(actual["U"]) and len(corners["UL"]):
        left_blocks.append(interp_pairs_k(actual["U"], corners["UL"], BOOST_STEPS, BOOST_MAX_DIST, BOOST_K_NEIGHBORS, BOOST_OVERSHOOT))
    if len(actual["D"]) and len(corners["DL"]):
        left_blocks.append(interp_pairs_k(actual["D"], corners["DL"], BOOST_STEPS, BOOST_MAX_DIST, BOOST_K_NEIGHBORS, BOOST_OVERSHOOT))
    if len(actual["L"]) and len(corners["UL"]):
        left_blocks.append(interp_pairs_k(actual["L"], corners["UL"], BOOST_STEPS//2, BOOST_MAX_DIST, BOOST_K_NEIGHBORS))
    if len(actual["L"]) and len(corners["DL"]):
        left_blocks.append(interp_pairs_k(actual["L"], corners["DL"], BOOST_STEPS//2, BOOST_MAX_DIST, BOOST_K_NEIGHBORS))
    left_interp = np.vstack(left_blocks) if left_blocks else np.empty((0,3))

    # --- BOOST RIGHT (new mirror logic) ---
    right_blocks = []
    if len(actual["U"]) and len(corners["UR"]):
        right_blocks.append(interp_pairs_k(actual["U"], corners["UR"], BOOST_STEPS, BOOST_MAX_DIST, BOOST_K_NEIGHBORS, BOOST_OVERSHOOT))
    if len(actual["D"]) and len(corners["DR"]):
        right_blocks.append(interp_pairs_k(actual["D"], corners["DR"], BOOST_STEPS, BOOST_MAX_DIST, BOOST_K_NEIGHBORS, BOOST_OVERSHOOT))
    if len(actual["R"]) and len(corners["UR"]):
        right_blocks.append(interp_pairs_k(actual["R"], corners["UR"], BOOST_STEPS//2, BOOST_MAX_DIST, BOOST_K_NEIGHBORS))
    if len(actual["R"]) and len(corners["DR"]):
        right_blocks.append(interp_pairs_k(actual["R"], corners["DR"], BOOST_STEPS//2, BOOST_MAX_DIST, BOOST_K_NEIGHBORS))
    right_interp = np.vstack(right_blocks) if right_blocks else np.empty((0,3))

    # --- combine ---
    interp_points = np.vstack([generic_interp, left_interp, right_interp]) if any(len(a) for a in [generic_interp, left_interp, right_interp]) else np.empty((0,3))

    print(f"✅ Actual points: {len(all_actual_points)}")
    print(f"✅ Corner points: {len(all_corners)}")
    print(f"✅ Interpolated total: {len(interp_points)} (Left: {len(left_interp)}, Right: {len(right_interp)})")

    visualize_combined(all_actual_points, all_corners, interp_points, drone_positions)

if __name__ == "__main__":
    main()
