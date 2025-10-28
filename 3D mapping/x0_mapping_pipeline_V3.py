"""
x0_mapping_pipeline_V3.py
-----------------
Corner-aware fusion of 4× ultrasonic + 4× ToF point clouds.

Fusion = ToF + Ultrasonic actuals + Extrapolated corners.
Open3D viewer shows the final fused cloud, colored by per-point weight.

Assumptions:
- Inputs share a frame; XY is the floor plane.
- Arrays may have extra columns (e.g., intensity). ensure_xyz() coerces to XYZ.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, QhullError
import open3d as o3d

from tof_map_V2 import main as tof_map   # (tof_points, traj_positions)
from us_map_V4  import main as us_map    # (us_interp_points, us_actual_points, us_corner_points)

def ensure_xyz(arr, xyz_cols=(0,1,2)):
    """Return an N×3 XYZ array. If only XY is present, pad Z with zeros."""
    
    if arr is None:
        return None
    A = np.asarray(arr)
    if A.ndim != 2 or A.shape[1] < 2:
        raise ValueError(f"Expected array with at least 2 columns, got shape {A.shape}")
    if A.shape[1] >= 3:
        cols = list(xyz_cols)
        return A[:, cols]
    else:
        pad_z = np.zeros((A.shape[0], 1), dtype=A.dtype)
        return np.hstack([A[:, :2], pad_z])

def build_room_polygon(us_corner_points_xy, max_vertices):
    """Build a CCW convex polygon from corner points and decimate to the max vertex count."""
    
    pts = np.asarray(us_corner_points_xy, dtype=float)
    if pts.shape[0] < 3:
        raise ValueError("Need ≥3 points to form a polygon")
    try:
        hull = ConvexHull(pts)
        poly = pts[hull.vertices]
    except QhullError:
        mn = pts.min(axis=0); mx = pts.max(axis=0)
        poly = np.array([[mn[0], mn[1]],[mx[0], mn[1]],[mx[0], mx[1]],[mn[0], mx[1]]], dtype=float)
    m = len(poly)
    if m > max_vertices:
        step = int(np.ceil(m / max_vertices))
        poly = poly[::step]
        
    return poly

def erode_convex_polygon(poly_xy, margin):
    """Shrink a convex polygon by moving vertices toward the centroid by a fixed margin."""
    
    poly = np.asarray(poly_xy, dtype=float)
    c = poly.mean(axis=0)
    radii = np.linalg.norm(poly - c, axis = 1)
    mean_r = max(radii.mean(), 1e-9)
    s = 1.0 - (margin / mean_r)
    s = max(0.0, min(1.0, s))
    
    return c + (poly - c) * s

def _edge_frames(poly_xy):
    """Compute per edge start points, tangents, and outward normals for a CCW polygon."""
    
    P = poly_xy
    Q = np.roll(P, -1, axis=0)
    E = Q - P
    T = E / (np.linalg.norm(E, axis = 1, keepdims = True) + 1e-12)
    N = np.stack([ T[:,1], -T[:,0] ], axis = 1)
    return P, N, T

def soft_clip_to_convex_polygon(points_xy, poly_xy, alpha, margin):
    """Soft clip points to a shrunken polygon. Outside points slide to the nearest edge then blend by alpha."""
    
    Ppoly = erode_convex_polygon(poly_xy, margin=margin)
    P, N, T = _edge_frames(Ppoly)

    X = np.asarray(points_xy, dtype=float)
    XN = X @ N.T
    PN = np.sum(P * N, axis=1)
    d  = XN - PN[None, :]

    max_d  = np.max(d, axis=1)
    outside = max_d > 0
    if not np.any(outside):
        return X.copy()

    d_out    = d[outside]
    edge_idx = np.argmax(d_out, axis=1)

    Pj = P[edge_idx]
    Nj = N[edge_idx]
    Tj = np.stack([-Nj[:,1], Nj[:,0]], axis=1)

    Xo = X[outside]
    t  = np.sum((Xo - Pj) * Tj, axis=1, keepdims=True)
    Q  = Pj + t * Tj

    X_new = X.copy()
    X_new[outside] = Q + alpha * (Xo - Q)
    
    return X_new

def fuse_ultra_tof_with_interp(tof_points, us_actual_points, us_corner_points, us_interp_points):
    """Fuse ToF, ultrasonic actuals, corners, and optional interpolation. Return fused points, weights, room polygon, and clipped ToF."""

    alpha_soft = 0.4
    margin=0.03
    w_tof=1.0
    w_ultra=0.6
    w_corner=0.25
    w_interp=0.45

    tof_xyz_cols=(0,1,2)
    us_actual_xyz_cols=(0,1,2)
    us_corner_xyz_cols=(0,1,2)
    us_interp_xyz_cols=(0,1,2)
    
    # Coerce to XYZ
    tof_xyz        = ensure_xyz(tof_points,        xyz_cols=tof_xyz_cols)
    us_actual_xyz  = ensure_xyz(us_actual_points,  xyz_cols=us_actual_xyz_cols)
    us_corner_xyz  = ensure_xyz(us_corner_points,  xyz_cols=us_corner_xyz_cols)
    us_interp_xyz  = ensure_xyz(us_interp_points,  xyz_cols=us_interp_xyz_cols) if us_interp_points is not None else None

    # Room polygon (convex) from corners, with safe fallback if <3 corners
    if us_corner_xyz is not None and us_corner_xyz.shape[0] >= 3:
        corners_xy = build_room_polygon(us_corner_xyz[:, :2], max_vertices=24)
        
    else:
        # Build an AABB from whatever exists: ultrasonic actuals, else ToF
        if us_actual_xyz is not None and len(us_actual_xyz) > 0:
            cand = us_actual_xyz[:, :2]
        else:
            cand = tof_xyz[:, :2]
        mn = cand.min(axis=0); mx = cand.max(axis=0)
        
        if np.allclose(mn, mx):
            # Degenerate: tiny square around the single point
            c = cand[0]; eps = 1e-3
            corners_xy = np.array([[c[0]-eps, c[1]-eps],
                                [c[0]+eps, c[1]-eps],
                                [c[0]+eps, c[1]+eps],
                                [c[0]-eps, c[1]+eps]], dtype=float)
        else:
            corners_xy = np.array([[mn[0], mn[1]],
                                [mx[0], mn[1]],
                                [mx[0], mx[1]],
                                [mn[0], mx[1]]], dtype=float)

    # Soft-clip ToF XY
    tof_clipped = tof_xyz.copy()
    tof_clipped[:, :2] = soft_clip_to_convex_polygon(tof_xyz[:, :2], corners_xy, alpha_soft, margin)

    # Stack with weights
    stacks = [tof_clipped, us_actual_xyz, us_corner_xyz]
    weights = [
        np.full(len(tof_clipped),  w_tof,    dtype=float),
        np.full(len(us_actual_xyz), w_ultra, dtype=float),
        np.full(len(us_corner_xyz), w_corner,dtype=float),
    ]
    
    if us_interp_xyz is not None and len(us_interp_xyz):
        stacks.append(us_interp_xyz)
        weights.append(np.full(len(us_interp_xyz), w_interp, dtype=float))

    fused_points  = np.vstack(stacks)
    fused_weights = np.concatenate(weights)

    return fused_points, fused_weights, corners_xy, tof_clipped

def visualise_ultrasonic_only_map(us_actual_points, us_corner_points, us_interp_points, traj_positions):
    """Open3D view of ultrasonic actuals, corners, and interpolation with trajectory and last pose marker."""
    
    geoms = []

    # Actual ultrasonic (pink)
    if us_actual_points is not None and len(us_actual_points):
        pc_actual = o3d.geometry.PointCloud()
        pc_actual.points = o3d.utility.Vector3dVector(np.asarray(us_actual_points))
        pc_actual.paint_uniform_color([1.0, 0.176, 0.667])
        geoms.append(pc_actual)

    # Corners (dark blue)
    if us_corner_points is not None and len(us_corner_points):
        pc_corners = o3d.geometry.PointCloud()
        pc_corners.points = o3d.utility.Vector3dVector(np.asarray(us_corner_points))
        pc_corners.paint_uniform_color([0.0, 0.44, 0.57])
        geoms.append(pc_corners)

    # Interpolation (light blue)
    if us_interp_points is not None and len(us_interp_points):
        pc_interp = o3d.geometry.PointCloud()
        pc_interp.points = o3d.utility.Vector3dVector(np.asarray(us_interp_points))
        pc_interp.paint_uniform_color([0.38, 0.78, 0.78])
        geoms.append(pc_interp)

    # Trajectory (pink line + last-pose sphere)
    if traj_positions is not None and len(traj_positions) > 1:
        T = np.asarray(traj_positions)
        traj = o3d.geometry.LineSet()
        traj.points = o3d.utility.Vector3dVector(T)
        traj.lines  = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(T) - 1)])
        traj.colors = o3d.utility.Vector3dVector([[1.0, 0.176, 0.667] for _ in range(len(T) - 1)])
        geoms.append(traj)

        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        sphere.translate(T[-1])
        sphere.paint_uniform_color([1.0, 0.176, 0.667])
        geoms.append(sphere)

    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2))
    o3d.visualization.draw_geometries(
        geoms,
        window_name="Ultrasonic Map with Extrapolation",
        width=1280, height=800,
        point_show_normal=False
    )

def visualise_stages(us_actual, us_corners, tof_clipped, fused_points, fused_weights, traj_positions, title_suffix):
    """Matplotlib figure with three panels. ToF after soft clip. Ultrasonic inputs. Fused cloud with optional weight coloring."""
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 6), constrained_layout=True)
    ax_tof, ax_us, ax_fused = axes

    # [1] ToF clipped
    ax = ax_tof
    ax.set_title("Stage 1 • ToF after soft-clip")
    if tof_clipped is not None and len(tof_clipped):
        ax.scatter(tof_clipped[:, 0], tof_clipped[:, 1], s=1, alpha=0.6, label="ToF clipped")
        
    ax.set_aspect("equal", "box"); ax.legend(loc="best")

    # [2] Ultrasonic
    ax = ax_us
    ax.set_title("Stage 2 • Ultrasonic inputs")
    if us_actual is not None and len(us_actual):
        ax.scatter(us_actual[:, 0], us_actual[:, 1], s=30, marker="x", label="Ultrasonic actual")
    if us_corners is not None and len(us_corners):
        ax.scatter(us_corners[:, 0], us_corners[:, 1], s=40, marker="s", label="Extrapolated corners")
    if traj_positions is not None and len(traj_positions):
        ax.plot(traj_positions[:, 0], traj_positions[:, 1], linewidth=1.0, label="Trajectory")
    ax.set_aspect("equal", "box"); ax.legend(loc="best")

    # [3] Fused cloud
    ax = ax_fused
    ax.set_title("Stage 3 • Fused cloud")
    if fused_points is not None and len(fused_points):
        if fused_weights is not None and len(fused_weights) == len(fused_points):
            sc = ax.scatter(fused_points[:, 0], fused_points[:, 1], s=2, c=fused_weights,
                            cmap="viridis", alpha=0.9, label="Fused points")
            cb = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
            cb.set_label("Per-point weight")
        else:
            ax.scatter(fused_points[:, 0], fused_points[:, 1], s=2, alpha=0.9, label="Fused points")
    ax.set_aspect("equal", "box"); ax.legend(loc="best")

    if title_suffix:
        fig.suptitle(title_suffix, fontsize=12)
    plt.show()

def visualise_open3d_final(points_xyz, weights, traj_positions):
    """Open3D final viewer with optional downsampling, gentle pull toward the trajectory, and weight based coloring."""
    
    pts = ensure_xyz(points_xyz).copy()
    w   = None if weights is None else np.asarray(weights, dtype=float)

    # Trajectory-based distance constraint
    if traj_positions is not None and len(traj_positions) > 0:
        T = ensure_xyz(traj_positions)
        pcd_traj = o3d.geometry.PointCloud()
        pcd_traj.points = o3d.utility.Vector3dVector(T)
        tree = o3d.geometry.KDTreeFlann(pcd_traj)

        pulled = 0
        for i in range(len(pts)):
            k, idx, _ = tree.search_knn_vector_3d(pts[i], 1)
            if k == 0:
                continue
            q = np.asarray(T[idx[0]])
            d_xy = np.linalg.norm(pts[i, :2] - q[:2])
            if d_xy > 1.5:
                pts[i, :2] = q[:2] + 0.3 * (pts[i, :2] - q[:2])  # pull 30% toward the path (XY only)
                pulled += 1
        print(f"Trajectory constraint pulled {pulled} points (> 1.5 m in XY).")

    # Colour mapping: low weight (light blue), high weight (dark blue)
    if w is not None and len(w) > 0:
        wmin, wmax = float(np.min(w)), float(np.max(w))
        t = np.zeros_like(w) if (wmax - wmin) < 1e-12 else (w - wmin) / (wmax - wmin)
        light = np.array([98/255, 200/255, 211/255])  # light blue
        dark  = np.array([0/255, 113/255, 145/255])   # dark blue
        colors = (light[None, :] * (1.0 - t[:, None])) + (dark[None, :] * t[:, None])
    else:
        colors = np.full((len(pts), 3), 0.8)

    geoms = []

    # Fused cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    geoms.append(pcd)

    # Trajectory (pink) and last pose marker
    if traj_positions is not None and len(traj_positions) > 1:
        Tfull = ensure_xyz(traj_positions)
        traj = o3d.geometry.LineSet()
        traj.points = o3d.utility.Vector3dVector(Tfull)
        traj.lines  = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(Tfull) - 1)])
        traj.colors = o3d.utility.Vector3dVector([[1.0, 0.176, 0.667] for _ in range(len(Tfull) - 1)])
        geoms.append(traj)

        marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        marker.translate(Tfull[-1])
        marker.paint_uniform_color([1.0, 0.176, 0.667])
        geoms.append(marker)

    o3d.visualization.draw_geometries(
        geoms,
        window_name="Fused and Filtered Cloud",
        width=1280, height=800,
        point_show_normal=False
    )

def cut_data(data_times, us_input_path, us_input_cropped):
    """Crop the ultrasonic CSV to the start and end times defined in data_times.csv."""
    
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
    print(f"Ultrasonic data cropped to {start_time:.2f}–{end_time:.2f}s")

def main():
    """Run cropping, generate ToF and ultrasonic maps, visualize ultrasonic only, fuse layers, and show 2D and 3D views."""
    
    t0 = time.time()
    
    data_name = "26_10_25_Lv4/3_LWF_both2/"
    base_path = "../optical_flow_method_data/combined_samples/" + data_name

    data_times = base_path + "data_times.csv"
    tof_input_cropped = base_path + "download_tof_cropped.csv"
    us_input_path = base_path + "Ultra_MB1030.csv"
    us_input_cropped = base_path + "us_cropped.csv"

    # Crop ultrasonic data
    cut_data(data_times, us_input_path, us_input_cropped)

    print("\n=== Stage 1: Generate ToF and Ultrasonic Maps ===")
    tof_points, traj_positions = tof_map(tof_input_cropped)
    us_interp_points, us_actual_points, us_corner_points = us_map(us_input_cropped)

    print(f"ToF points: {len(tof_points)} | US actual: {len(us_actual_points)} | US corners: {len(us_corner_points)}")
    if us_interp_points is not None:
        print(f"(Ultrasonic interpolation present: {len(us_interp_points)} pts)")
    t1 = time.time()
    print(f"Stage 1 time: {t1 - t0:.2f}s")

    # Ultrasonic Map with Extrapolation
    print("\nUltrasonic-only map with extrapolation")
    visualise_ultrasonic_only_map(us_actual_points, us_corner_points, us_interp_points, traj_positions)

    # Fusion with interpolation
    print("\n=== Stage 2: Corner-aware fusion (US only) ===")
    t2 = time.time()
    fused, w_fused, _, tof_clipped = fuse_ultra_tof_with_interp(tof_points, us_actual_points, us_corner_points, us_interp_points)

    print(f"Fused points: {len(fused)} | Fusion time: {time.time() - t2:.2f}s")

    # 2D Visualisation
    tv0 = time.time()
    visualise_stages(us_actual_points, us_corner_points, tof_clipped, fused, w_fused, traj_positions, "Ultrasonic and ToF Fusion")

    print(f"2D Visualisation time: {time.time() - tv0:.2f}s")

    # Open3D viewer of the final fused cloud)
    print("Launching Open3D viewer… (close the window to continue)")
    visualise_open3d_final(fused, w_fused, traj_positions)
    
    print(f"\nTotal time: {time.time() - t0:.2f}s")

if __name__ == "__main__":
    main()
