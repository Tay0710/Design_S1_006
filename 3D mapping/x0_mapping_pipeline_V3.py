"""
x0_mapping_pipeline_V3.py
-----------------
Corner-aware fusion of 4× ultrasonic + 4× ToF point clouds.

Fusion = ToF + Ultrasonic actuals + Extrapolated corners.
Open3D viewer shows the final fused cloud, colored by per-point weight.

Assumptions:
- Inputs share a frame; XY is the floor plane.
- Arrays may have extra columns (e.g., intensity). ensure_xyz() coerces to XYZ.
- You have:
    from tof_map_V2 import main as tof_map        # -> (tof_points, traj_positions)
    from us_map_V3  import main as us_map         # -> (us_interp_points, us_actual_points, us_corner_points)
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, QhullError

# --- Optional, for Open3D visualization ---
try:
    import open3d as o3d
    O3D_AVAILABLE = True
except Exception:
    O3D_AVAILABLE = False

# Project imports
from tof_map_V2 import main as tof_map   # -> (tof_points, traj_positions)
from us_map_V4  import main as us_map    # -> (us_interp_points, us_actual_points, us_corner_points)

# =============================================================================
# Utilities
# =============================================================================

def ensure_xyz(arr: np.ndarray, xyz_cols=(0, 1, 2)) -> np.ndarray:
    """Coerce any array to Nx3 XYZ. If only XY, pad Z=0."""
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

def _downsample(points: np.ndarray, max_n: int) -> np.ndarray:
    """Randomly downsample for plotting speed (does not affect data)."""
    if points is None:
        return None
    n = len(points)
    if n <= max_n:
        return points
    idx = np.random.default_rng(42).choice(n, size=max_n, replace=False)
    return points[idx]

# =============================================================================
# Geometry & Soft-clip (XY)
# =============================================================================

def build_room_polygon(us_corner_points_xy: np.ndarray, max_vertices: int = 24) -> np.ndarray:
    """Convex hull of corners (CCW), decimated to <= max_vertices. Fallback: AABB."""
    pts = np.asarray(us_corner_points_xy, dtype=float)
    if pts.shape[0] < 3:
        raise ValueError("Need ≥3 points to form a polygon")
    try:
        hull = ConvexHull(pts)
        poly = pts[hull.vertices]  # CCW
    except QhullError:
        mn = pts.min(axis=0); mx = pts.max(axis=0)
        poly = np.array([[mn[0], mn[1]],[mx[0], mn[1]],[mx[0], mx[1]],[mn[0], mx[1]]], dtype=float)
    m = len(poly)
    if m > max_vertices:
        step = int(np.ceil(m / max_vertices))
        poly = poly[::step]
    return poly

def erode_convex_polygon(poly_xy: np.ndarray, margin: float = 0.03) -> np.ndarray:
    """Shrink convex polygon by moving vertices toward centroid by a fixed radial margin."""
    poly = np.asarray(poly_xy, dtype=float)
    c = poly.mean(axis=0)
    radii = np.linalg.norm(poly - c, axis=1)
    mean_r = max(radii.mean(), 1e-9)
    s = 1.0 - (margin / mean_r)
    s = max(0.0, min(1.0, s))
    return c + (poly - c) * s

def _edge_frames(poly_xy: np.ndarray):
    """Edge starts P, outward normals N, tangents T for CCW convex polygon."""
    P = poly_xy
    Q = np.roll(P, -1, axis=0)
    E = Q - P
    T = E / (np.linalg.norm(E, axis=1, keepdims=True) + 1e-12)
    N = np.stack([ T[:,1], -T[:,0] ], axis=1)  # outward normal
    return P, N, T

def soft_clip_to_convex_polygon(points_xy: np.ndarray,
                                poly_xy: np.ndarray,
                                alpha: float = 0.4,
                                margin: float = 0.03) -> np.ndarray:
    """
    Vectorized soft-clip of points to a convex polygon (CCW).
    alpha in [0..1]: 0=snap to edge, 1=no change.
    """
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

# =============================================================================
# Fusion (no ultrasonic interpolation)
# =============================================================================

def fuse_ultra_tof_no_interp(
    tof_points: np.ndarray,
    us_actual_points: np.ndarray,
    us_corner_points: np.ndarray,
    alpha_soft=0.4,
    margin=0.03,
    w_tof=1.0,
    w_ultra=0.6,
    w_corner=0.25,
    # column picks
    tof_xyz_cols=(0,1,2),
    us_actual_xyz_cols=(0,1,2),
    us_corner_xyz_cols=(0,1,2),
):
    """
    Fuse ToF + Ultrasonic + Corner priors.
    Interpolated ultrasonic data is NOT used in this function.
    Returns:
      fused_points (M,3), fused_weights (M,), corners_xy (K,2), tof_clipped (N,3)
    """
    # Coerce to XYZ
    tof_xyz       = ensure_xyz(tof_points,       xyz_cols=tof_xyz_cols)
    us_actual_xyz = ensure_xyz(us_actual_points, xyz_cols=us_actual_xyz_cols)
    us_corner_xyz = ensure_xyz(us_corner_points, xyz_cols=us_corner_xyz_cols)

    # Room polygon (convex) from corners
    corners_xy = build_room_polygon(us_corner_xyz[:, :2], max_vertices=24)

    # Soft-clip ToF XY (Z untouched)
    tof_clipped = tof_xyz.copy()
    tof_clipped[:, :2] = soft_clip_to_convex_polygon(
        tof_xyz[:, :2], corners_xy, alpha=alpha_soft, margin=margin
    )

    # Stack with weights
    fused_points = np.vstack([tof_clipped, us_actual_xyz, us_corner_xyz])
    fused_weights = np.concatenate([
        np.full(len(tof_clipped), w_tof, dtype=float),
        np.full(len(us_actual_xyz), w_ultra, dtype=float),
        np.full(len(us_corner_xyz), w_corner, dtype=float),
    ])
    return fused_points, fused_weights, corners_xy, tof_clipped

# =============================================================================
# Matplotlib 2D visualization
# =============================================================================

def _plot_room_polygon(ax, poly_xy, **kwargs):
    poly = np.asarray(poly_xy)
    loop = np.vstack([poly, poly[0]])
    ax.plot(loop[:, 0], loop[:, 1], **kwargs)

def visualize_stages(
    tof_raw,
    us_actual,
    us_corners,
    corners_xy,
    tof_clipped,
    fused_points,
    fused_weights,
    traj_positions=None,
    title_suffix=""
):
    """
    2×2 subplot:
      (1,1) Inputs (no ultrasonic interpolation shown)
      (1,2) ToF clipped
      (2,1) Fused (US only)
      (2,2) Fused (US only)  — repeated for symmetric layout
    """
    # Downsample for plotting
    tof_raw_plot  = _downsample(tof_raw,   20000)
    fused_plot    = _downsample(fused_points, 40000)
    w_plot        = _downsample(fused_weights, 40000)
    tof_clip_plot = _downsample(tof_clipped, 20000) if tof_clipped is not None else None

    fig, axes = plt.subplots(2, 2, figsize=(12, 10), constrained_layout=True)
    ax11, ax12, ax21, ax22 = axes.ravel()

    # (1,1) Inputs (no interp)
    ax = ax11
    ax.set_title("Stage 0 • Inputs (Top-down XY) — no ultrasonic interpolation")
    if tof_raw_plot is not None and len(tof_raw_plot):
        ax.scatter(tof_raw_plot[:, 0], tof_raw_plot[:, 1], s=1, alpha=0.35, label="ToF raw")
    if us_actual is not None and len(us_actual):
        ax.scatter(us_actual[:, 0], us_actual[:, 1], s=30, marker="x", label="Ultrasonic actual")
    if us_corners is not None and len(us_corners):
        ax.scatter(us_corners[:, 0], us_corners[:, 1], s=40, marker="s", label="Extrapolated corners")
    _plot_room_polygon(ax, corners_xy, linewidth=1.0)
    if traj_positions is not None and len(traj_positions):
        ax.plot(traj_positions[:, 0], traj_positions[:, 1], linewidth=1.0, label="Trajectory")
    ax.set_aspect("equal", "box"); ax.legend(loc="best")

    # (1,2) ToF clipped
    ax = ax12
    ax.set_title("Stage 1 • ToF after corner-aware soft-clip")
    if tof_clip_plot is not None and len(tof_clip_plot):
        ax.scatter(tof_clip_plot[:, 0], tof_clip_plot[:, 1], s=1, alpha=0.6, label="ToF clipped")
    _plot_room_polygon(ax, corners_xy, linewidth=1.0)
    ax.set_aspect("equal", "box"); ax.legend(loc="best")

    # (2,1) Fused (US only)
    ax = ax21
    ax.set_title("Stage 2 • Fused cloud (US only)")
    if fused_plot is not None and len(fused_plot):
        sc = ax.scatter(fused_plot[:, 0], fused_plot[:, 1], s=2, c=w_plot,
                        cmap="viridis", alpha=0.9, label="Fused points")
        cb = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04); cb.set_label("Per-point weight")
    _plot_room_polygon(ax, corners_xy, linewidth=1.0)
    ax.set_aspect("equal", "box"); ax.legend(loc="best")

    # (2,2) Repeat fused for symmetric layout
    ax = ax22
    ax.set_title("Stage 2 • Fused cloud (US only)")
    if fused_plot is not None and len(fused_plot):
        sc = ax.scatter(fused_plot[:, 0], fused_plot[:, 1], s=2, c=w_plot,
                        cmap="viridis", alpha=0.9, label="Fused points")
        cb = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04); cb.set_label("Per-point weight")
    _plot_room_polygon(ax, corners_xy, linewidth=1.0)
    ax.set_aspect("equal", "box"); ax.legend(loc="best")

    if title_suffix:
        fig.suptitle(title_suffix, fontsize=12)
    plt.show()

# =============================================================================
# Open3D Visualization (final fused result)
# =============================================================================

def _make_polygon_lineset(poly_xy: np.ndarray, z: float, color=(1.0, 0.4, 0.1)):
    """Open3D LineSet for a single polygon ring at height z."""
    poly = np.asarray(poly_xy, dtype=float)
    n = len(poly)
    pts = np.c_[poly, np.full(n, z)]
    lines = np.array([[i, (i + 1) % n] for i in range(n)], dtype=np.int32)

    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(pts)
    ls.lines  = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector(np.tile(color, (len(lines), 1)))
    return ls

def visualize_open3d_final(points_xyz: np.ndarray,
                           weights: np.ndarray | None,
                           corners_xy: np.ndarray | None = None,
                           traj_positions: np.ndarray | None = None,
                           max_points: int = 150_000,
                           title: str = "Fused cloud (Open3D) — US only"):
    """
    Show the final fused cloud in Open3D, colored by per-point weight.
    Overlays footprint wireframe at median Z and the drone trajectory (magenta + sphere).
    """
    if not O3D_AVAILABLE:
        print("⚠️ Open3D not installed. Run: pip install open3d")
        return

    pts = ensure_xyz(points_xyz)
    if pts.shape[0] > max_points:
        idx = np.random.default_rng(0).choice(pts.shape[0], size=max_points, replace=False)
        pts = pts[idx]
        weights = weights[idx] if weights is not None else None

    # Colors: weight→viridis, else gray
    if weights is not None:
        w = np.asarray(weights, dtype=float)
        wmin, wmax = float(np.min(w)), float(np.max(w))
        wnorm = np.zeros_like(w) if (wmax - wmin) < 1e-12 else (w - wmin) / (wmax - wmin)
        cmap = plt.cm.get_cmap("viridis")
        colors = cmap(wnorm)[:, :3]
    else:
        colors = np.full((len(pts), 3), 0.8)

    geoms = []

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    geoms.append(pcd)

    # Room footprint wireframe at median Z
    if corners_xy is not None and len(corners_xy) >= 3:
        z_mid = float(np.median(pts[:, 2]))
        geoms.append(_make_polygon_lineset(corners_xy, z=z_mid, color=(1.0, 0.4, 0.1)))

    # Trajectory (magenta) + last pose marker
    if traj_positions is not None and len(traj_positions) > 1:
        traj_positions = ensure_xyz(traj_positions)
        traj = o3d.geometry.LineSet()
        traj.points = o3d.utility.Vector3dVector(traj_positions)
        traj.lines  = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(traj_positions) - 1)])
        traj.colors = o3d.utility.Vector3dVector([[1.0, 0.176, 0.667] for _ in range(len(traj_positions) - 1)])
        geoms.append(traj)

        marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        marker.translate(traj_positions[-1])
        marker.paint_uniform_color([1.0, 0.176, 0.667])
        geoms.append(marker)

    o3d.visualization.draw_geometries(
        geoms,
        window_name=title,
        width=1280, height=800,
        point_show_normal=False
    )

# =============================================================================
# Main
# =============================================================================

# === Stage 1: Crop ultrasonic dataset ===
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
    print(f"✔ Ultrasonic data cropped to {start_time:.2f}–{end_time:.2f}s")

def main():
    t0 = time.time()

    # Paths
    data_name = "23_10_25_MILC/"
    base_path = "../optical_flow_method_data/combined_samples/" + data_name

    data_times = base_path + "data_times.csv"
    tof_input_cropped = base_path + "download_tof_cropped.csv"
    us_input_path = base_path + "Ultra_MB1030.csv"
    us_input_cropped = base_path + "us_cropped.csv"

    # Crop ultrasonic (if needed)
    cut_data(data_times, us_input_path, us_input_cropped)

    # Generate maps
    print("\n=== Stage 1: Generate ToF and Ultrasonic Maps ===")
    tof_points, traj_positions = tof_map(tof_input_cropped)
    us_interp_points, us_actual_points, us_corner_points = us_map(us_input_cropped)  # interp returned but IGNORED below

    print(f"ToF points: {len(tof_points)} | US actual: {len(us_actual_points)} | US corners: {len(us_corner_points)}")
    if us_interp_points is not None:
        print(f"(Ultrasonic interpolation present: {len(us_interp_points)} pts)  →  IGNORED by this pipeline")
    t1 = time.time()
    print(f"Stage 1 time: {t1 - t0:.2f}s")

    # Fusion (no interpolation)
    print("\n=== Stage 2: Corner-aware fusion (US only) ===")
    t2 = time.time()
    fused, w_fused, room_poly_xy, tof_clipped = fuse_ultra_tof_no_interp(
        tof_points=tof_points,
        us_actual_points=us_actual_points,
        us_corner_points=us_corner_points,
        alpha_soft=0.4,   # 0=snap, try 0.25–0.5
        margin=0.03,      # polygon erosion
        w_tof=1.0,
        w_ultra=0.6,
        w_corner=0.25,
        tof_xyz_cols=(0,1,2),
        us_actual_xyz_cols=(0,1,2),
        us_corner_xyz_cols=(0,1,2),
    )
    print(f"Fused points: {len(fused)} | Fusion time: {time.time() - t2:.2f}s")

    # 2D Visualization
    tv0 = time.time()
    visualize_stages(
        tof_raw=ensure_xyz(tof_points),
        us_actual=ensure_xyz(us_actual_points),
        us_corners=ensure_xyz(us_corner_points),
        corners_xy=room_poly_xy,
        tof_clipped=tof_clipped,
        fused_points=fused,
        fused_weights=w_fused,
        traj_positions=traj_positions,
        title_suffix="Ultrasonic + ToF Fusion (no ultrasonic interpolation)"
    )
    print(f"2D Visualization time: {time.time() - tv0:.2f}s")

    # Open3D viewer (final fused cloud)
    if O3D_AVAILABLE:
        print("Launching Open3D viewer… (close the window to continue)")
        visualize_open3d_final(
            points_xyz=fused,
            weights=w_fused,
            corners_xy=room_poly_xy,
            traj_positions=traj_positions,
            max_points=150_000,
            title="Fused cloud — ToF + Ultrasonic (no interpolation)"
        )
    else:
        print("⚠️ Open3D not installed. To enable the 3D viewer: pip install open3d")

    print(f"\nTotal time: {time.time() - t0:.2f}s")

if __name__ == "__main__":
    main()
