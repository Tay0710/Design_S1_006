"""
us_map_V4.py
------------------------------
Secondary map of the ELEC5550 Indoor 3D Mapping Design Project (2025) mapping pipeline.

Purpose:
    Build world-frame ultrasonic point sets for fusion including: actual U/L/R points,
    synthetic floor (Down) points, and corner and interpolated fills (UR, UL, DR, DL) for map completion.

Overview:
    Transforms ultrasonic readings into world coordinates using the IMU
    rotation matrices and the drone trajectory. Keeps Up points above a minimum
    ceiling distance, adds synthetic floor points, computes time-aligned
    corner points, and generates boosted left and right interpolations.
    Opens an Open3D viewer that shows only actual points, floor and trajectory.

    Notes: ultrasonic distances are parsed in centimetres and converted to metres.

Usage:
    Called from x0_mapping_pipeline_V2.py to supply the ultrasonic data points and extrapolated map.

Inputs:
    us_input_cropped (argument supplied to main)
        Columns:
            time, type, distance
    ../optical_flow_method_data/rotation_matrices.csv
        Columns:
            time, r00, r01, r02, r10, r11, r12, r20, r21, r22
    ../optical_flow_method_data/xy_velocities.csv
        Columns:
            time (s), v_x (m/s), v_y (m/s), pos_z (m)
    ../optical_flow_method_data/xy_velocities_to_world_frame.csv
        Columns:
            time (s), pos_world_x, pos_world_y, pos_world_z

Outputs:
    Returns:
        interp_points      (N×3) interpolated wall-fill points
        us_actual_points   (M×3) actual U/L/R and synthetic floor points
        us_corner_points   (K×3) UR/UL/DR/DL corner points
"""

import numpy as np
import pandas as pd
import open3d as o3d
from scipy.spatial import cKDTree

# Sensor offsets
offsetU = np.array([0.0, 0.0, 0.0])
offsetD = np.array([0.0, 0.0, 0.0])
offsetL = np.array([0.0, 0.0, 0.0])
offsetR = np.array([0.0, 0.0, 0.0])

# Tolerance for time between sensor readings
TIME_TOL = 0.055

# Ceiling acceptance threshold (metres): keep Up only if distance >= 0.5 m
ROOF_MIN_DIST_M = 0.5

# Boosted wall fills (left and right)
BOOST_STEPS = 28
BOOST_MAX_DIST = 2.0
BOOST_K_NEIGHBORS = 2
BOOST_OVERSHOOT = 0.10

def load_rotation_matrices(rot_csv):
    """Load [time, r00..r22] and reshape to (N,3,3) rotation matrices."""
    
    rot = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=range(1, 10))
    times = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=(0,))
    
    return times, rot.reshape(-1, 3, 3)

def parse_distance(v):
    """Parse ultrasonic distance in cm and return metres. If invalid return NaN."""
    try:
        val = float(v)
        if val <= 0:
            return np.nan
        return val / 100.0
    except:
        return np.nan

def to_world(local_vec, rot_mat, drone_pos):
    """Transform local vector by R and translate by drone position."""
    
    return drone_pos + rot_mat @ local_vec

def create_floor(up_times, traj_time, rot_mats, drone_positions):
    """Build floor points (z=0) and synthetic D rows (cm) for kept Up times."""
    
    floor_world = []
    drows = []

    for t in up_times:
        idx = np.searchsorted(traj_time, t, side="right")
        if idx >= len(traj_time):
            continue
        
        rot_mat = rot_mats[min(idx, len(rot_mats) - 1)]
        drone_pos = drone_positions[idx]

        p0  = drone_pos + rot_mat @ offsetD
        dir = rot_mat @ np.array([0.0, 0.0, -1.0])

        floor_world.append(np.array([p0[0], p0[1], 0.0]))

        if abs(dir[2]) >= 1e-9:
            s = -p0[2] / dir[2]
            if np.isfinite(s) and s > 0:
                drows.append((float(t), "D", float(s * 100.0)))

    floor_world = np.array(floor_world) if len(floor_world) else np.empty((0,3))
    down_rows = pd.DataFrame(drows, columns=["time", "type", "distance"]) if drows else \
                pd.DataFrame(columns=["time", "type", "distance"])
    
    return floor_world, down_rows

def visualize_actual_only(actual_U, actual_L, actual_R, floor_D, drone_positions):
    """Open3D view: actual U/L/R, synthetic floor, trajectory, axes."""
    
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
    vis.create_window(window_name="Ultrasonic Map without Extrapolation")
    
    for g in geoms:
        vis.add_geometry(g)
        
    opt = vis.get_render_option()
    opt.point_size = 5.5
    vis.run()
    vis.destroy_window()

def align_pair(df, t1, t2, tol=TIME_TOL):
    """Align two sensor streams by nearest timestamps within tolerance."""
    
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

def interp_pairs(points_a, points_b, steps, overshoot=0.0):
    """For each A, interpolate a straight line to its single nearest B."""
    
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
    """For each A, interpolate lines to up to k nearest Bs within max_dist."""
    
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

def main(us_input_cropped):
    """Build actual U/L/R, synthesize floor, compute corners and interpolation, visualize, return arrays."""

    traj = pd.read_csv("../optical_flow_method_data/xy_velocities_to_world_frame.csv")
    vel  = pd.read_csv("../optical_flow_method_data/xy_velocities.csv")
    times_mat, rot_mats = load_rotation_matrices("../optical_flow_method_data/rotation_matrices.csv")
    us = pd.read_csv(us_input_cropped)

    # Build drone trajectory positions
    traj_time = traj["time (s)"].values
    drone_positions = []
    for i in range(len(traj)):
        drone_pos = (
            traj["pos_world_x"].iloc[i],
            traj["pos_world_y"].iloc[i],
            traj["pos_world_z"].iloc[i],
        )
        drone_positions.append(drone_pos)
    drone_positions = np.array(drone_positions)

    actual_U, actual_L, actual_R = [], [], []
    us["type"] = us["type"].astype(str).str.strip().str.upper()
    print(us["type"])

    vel_y = vel["v_y (m/s)"].rolling(window=5).mean()

    up_times = []

    for _, row in us.iterrows():
        t = row["time"]
        d = parse_distance(row["distance"])
        s = row["type"]

        # Require a valid reading and one of the expected sensor types
        if s not in {"U", "L", "R"} or np.isnan(d):
            continue

        # Apply ceiling distance gate: keep Up only if >= ROOF_MIN_DIST_M
        if s == "U" and d < ROOF_MIN_DIST_M:
            continue

        idx = np.searchsorted(traj_time, t, side="right")
        if idx >= len(traj_time):
            continue

        # Get nearest rotation matrix by time
        rot_idx = np.searchsorted(times_mat, t, side="right")
        rot_mat = rot_mats[min(rot_idx, len(rot_mats) - 1)]
        drone_pos = drone_positions[idx]

        vel_y_u = vel_y[min(idx, len(rot_mats) - 1)]

        if abs(vel_y_u) > 0.5:
            if s == "U":
                v = np.array([0, 0, d]) + offsetU
                up_times.append(t)
                actual_U.append(to_world(v, rot_mat, drone_pos))
            elif s == "L":
                v = np.array([ d, 0, 0]) + offsetL
                actual_L.append(to_world(v, rot_mat, drone_pos))
            elif s == "R":
                v = np.array([-d, 0, 0]) + offsetR
                actual_R.append(to_world(v, rot_mat, drone_pos))

    actual_U = np.array(actual_U) if len(actual_U) else np.empty((0,3))
    actual_L = np.array(actual_L) if len(actual_L) else np.empty((0,3))
    actual_R = np.array(actual_R) if len(actual_R) else np.empty((0,3))

    # Synthesise floor (Down) world points + synthetic D rows (cm)
    floor_D, down_rows = create_floor(up_times, traj_time, rot_mats, drone_positions)

    # Make a copy of the ultrasonic dataframe that includes synthetic D rows for corner building
    us_with_D = pd.concat([us, down_rows], ignore_index=True)
    us_with_D["type"] = us_with_D["type"].astype(str).str.strip().str.upper()

    print(f"Up points (kept):   {len(actual_U)}  (threshold >= {ROOF_MIN_DIST_M:.2f} m)")
    print(f"Left points:        {len(actual_L)}")
    print(f"Right points:       {len(actual_R)}")
    print(f"Floor (D*):         {len(floor_D)}")

    # Prepare per-sensor actual dictionary expected by interpolation steps
    actual = {
        "U": actual_U,
        "D": floor_D,
        "L": actual_L,
        "R": actual_R,
    }
    
    all_actual_points = np.vstack([a for a in actual.values() if len(a)]) if any(len(a) for a in actual.values()) else np.empty((0,3))

    # Extrapolate corners
    pairs = {"UR": ("U", "R"), "UL": ("U", "L"), "DR": ("D", "R"), "DL": ("D", "L")}
    corners = {k: [] for k in pairs}

    for key, (a, b) in pairs.items():
        aligned = align_pair(us_with_D, a, b, TIME_TOL)
        if aligned.empty:
            continue
        
        for _, r in aligned.iterrows():
            t = r["time"]; dA, dB = r[a], r[b]
            if np.isnan(dA) or np.isnan(dB):
                continue
            
            idx = np.searchsorted(traj_time, t, side="right")
            if idx >= len(traj_time):
                continue
            
            rot_idx = np.searchsorted(times_mat, t, side="right")
            rot_mat = rot_mats[min(rot_idx, len(rot_mats) - 1)]
            drone_pos = drone_positions[idx]

            if a == "U": vA = np.array([0, 0,  dA]) + offsetU
            elif a == "D": vA = np.array([0, 0, -dA]) + offsetD
            if b == "R": vB = np.array([-dB, 0, 0]) + offsetR
            elif b == "L": vB = np.array([ dB, 0, 0]) + offsetL

            corner_local = np.array([vB[0], 0.0, vA[2]])
            corners[key].append(to_world(corner_local, rot_mat, drone_pos))

    for k in corners:
        corners[k] = np.array(corners[k]) if len(corners[k]) else np.empty((0,3))
    all_corners = np.vstack([a for a in corners.values() if len(a)]) if any(len(a) for a in corners.values()) else np.empty((0,3))

    # Boosted left interpolation (UL / DL)
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

    # Boosted left interpolation (UR / DR)
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

    # Combine the interpolations
    interp_points = (
        np.vstack([left_interp, right_interp])
        if (len(left_interp) or len(right_interp))
        else np.empty((0, 3))
    )

    print(f"Corner points: {len(all_corners)}")
    print(f"Interpolated total: {len(interp_points)} (Left: {len(left_interp)}, Right: {len(right_interp)})")

    visualize_actual_only(actual_U, actual_L, actual_R, floor_D, drone_positions)

    us_actual_points = np.vstack([a for a in [actual_U, actual_L, actual_R, floor_D] if len(a)]) \
                       if any(len(a) for a in [actual_U, actual_L, actual_R, floor_D]) else np.empty((0,3))
    us_corner_points = all_corners

    return interp_points, us_actual_points, us_corner_points

if __name__ == "__main__":
    main()
