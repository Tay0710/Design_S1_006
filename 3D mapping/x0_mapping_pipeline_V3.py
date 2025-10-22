"""
x0_mapping_pipeline_V3.py
-----------------
Corner-aware fusion of 4× ultrasonic + 4× ToF point clouds.

What you get:
1) Soft clamp of ToF points to a compact room polygon from your extrapolated corners.
2) Fused cloud with per-point weights:
   - ToF = 1.0 (surface truth)
   - Ultrasonic actuals ≈ 0.6 (constraints)
   - Corner priors ≈ 0.25 (soft)
   - Optional ultrasonic interpolation ≈ 0.35 (soft)
3) 2×2 subplots: Inputs • Clipped ToF • Fused (interp ON) • Fused (interp OFF)

Assumptions:
- Inputs are in a common frame; XY is the floor plane.
- Arrays may have extra columns (e.g., intensity, confidence). We coerce to XYZ via ensure_xyz().
- You have:
    from tof_map_V2 import main as tof_map        # -> (tof_points, traj_positions)
    from us_map_V3  import main as us_map         # -> (us_interp_points, us_actual_points, us_corner_points)
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, QhullError  # convex hull + robust fallback

# Project imports (keep as-is in your repo)
from tof_map_V2 import main as tof_map   # returns: (tof_points, traj_positions)
from us_map_V3  import main as us_map    # returns: (us_interp_points, us_actual_points, us_corner_points)

# =============================================================================
# Utilities: coercion + plotting downsample (plotting does NOT affect fusion)
# =============================================================================

def ensure_xyz(arr: np.ndarray, xyz_cols=(0, 1, 2)) -> np.ndarray:
    """
    Coerce an array to Nx3 XYZ by selecting the given columns or padding Z=0 if only XY present.
    - If arr has >=3 columns -> take columns xyz_cols.
    - If arr has exactly 2 columns -> pad a zero Z.
    - If arr has 1 or 0 columns -> raises.
    """
    if arr is None:
        return None
    A = np.asarray(arr)
    if A.ndim != 2 or A.shape[1] < 2:
        raise ValueError(f"Expected array with at least 2 columns, got shape {A.shape}")
    if A.shape[1] >= 3:
        cols = list(xyz_cols)
        return A[:, cols]
    else:
        # pad Z = 0
        pad_z = np.zeros((A.shape[0], 1), dtype=A.dtype)
        return np.hstack([A[:, :2], pad_z])

def _downsample(points: np.ndarray, max_n: int) -> np.ndarray:
    """Randomly downsample a point set to at most max_n points (plotting only)."""
    if points is None:
        return None
    n = len(points)
    if n <= max_n:
        return points
    idx = np.random.default_rng(42).choice(n, size=max_n, replace=False)
    return points[idx]

# =============================================================================
# Geometry: compact convex polygon + vectorized soft-clipping (fast)
# =============================================================================

def build_room_polygon(us_corner_points_xy: np.ndarray, max_vertices: int = 24) -> np.ndarray:
    """
    Build a compact convex polygon from many 'corner' points:
      1) Convex hull in CCW order,
      2) Decimate hull vertices to <= max_vertices to keep clipping cheap.
    Fallback: if hull fails/degenerate, use the bounding box of the points.
    """
    pts = np.asarray(us_corner_points_xy, dtype=float)
    if pts.shape[0] < 3:
        raise ValueError("Need ≥3 points to form a polygon")

    try:
        hull = ConvexHull(pts)
        poly = pts[hull.vertices]  # CCW
    except QhullError:
        mn = pts.min(axis=0)
        mx = pts.max(axis=0)
        poly = np.array([[mn[0], mn[1]],
                         [mx[0], mn[1]],
                         [mx[0], mx[1]],
                         [mn[0], mx[1]]], dtype=float)

    m = len(poly)
    if m > max_vertices:
        step = int(np.ceil(m / max_vertices))
        poly = poly[::step]

    return poly

def erode_convex_polygon(poly_xy: np.ndarray, margin: float = 0.03) -> np.ndarray:
    """
    Erode (shrink) a convex polygon by moving vertices toward centroid
    by a fixed radial margin. Simple tightening so clipping is stricter.
    """
    poly = np.asarray(poly_xy, dtype=float)
    c = poly.mean(axis=0)
    radii = np.linalg.norm(poly - c, axis=1)
    mean_r = max(radii.mean(), 1e-9)
    s = 1.0 - (margin / mean_r)
    s = max(0.0, min(1.0, s))
    return c + (poly - c) * s

def _edge_frames(poly_xy: np.ndarray):
    """
    For a CCW convex polygon, compute:
      P: edge start points (M,2)
      N: outward unit normals (M,2)
      T: unit tangents (M,2)
    """
    P = poly_xy
    Q = np.roll(P, -1, axis=0)
    E = Q - P
    T = E / (np.linalg.norm(E, axis=1, keepdims=True) + 1e-12)   # unit tangent
    N = np.stack([ T[:,1], -T[:,0] ], axis=1)                    # outward normal (CCW)
    return P, N, T

def soft_clip_to_convex_polygon(points_xy: np.ndarray,
                                poly_xy: np.ndarray,
                                alpha: float = 0.4,
                                margin: float = 0.03) -> np.ndarray:
    """
    Vectorized soft-clip of points to a convex polygon (CCW).

    Steps:
      - Erode polygon by 'margin'.
      - Compute signed distances: d_ij = X_i·N_j - P_j·N_j  (shape (N,M)).
      - A point is outside if max_j d_ij > 0. For each outside point, pick the
        most violated edge j*, project to that edge line, then pull by 'alpha':
            new = Q + alpha * (x - Q)
    Returns: (N,2) clipped XY
    """
    Ppoly = erode_convex_polygon(poly_xy, margin=margin)
    P, N, T = _edge_frames(Ppoly)            # (M,2) each

    X = np.asarray(points_xy, dtype=float)   # (N,2)
    assert X.ndim == 2 and X.shape[1] == 2, "points_xy must be (N,2)"

    # Signed distances (N,M): d_ij = X_i·N_j - P_j·N_j
    XN = X @ N.T                 # (N,M)
    PN = np.sum(P * N, axis=1)   # (M,)
    d  = XN - PN[None, :]        # (N,M)

    max_d  = np.max(d, axis=1)   # (N,)
    outside = max_d > 0
    if not np.any(outside):
        return X.copy()

    # Choose active edge for outside points
    d_out    = d[outside]                  # (No,M)
    edge_idx = np.argmax(d_out, axis=1)    # (No,)

    Pj = P[edge_idx]                       # (No,2)
    Nj = N[edge_idx]                       # (No,2)
    Tj = np.stack([-Nj[:,1], Nj[:,0]], axis=1)  # (No,2)

    Xo = X[outside]                        # (No,2)
    t  = np.sum((Xo - Pj) * Tj, axis=1, keepdims=True)  # (No,1)
    Q  = Pj + t * Tj                       # (No,2)

    X_new = X.copy()
    X_new[outside] = Q + alpha * (Xo - Q)
    return X_new

# =============================================================================
# Fusion core
# =============================================================================

def fuse_ultra_tof(
    tof_points: np.ndarray,
    us_actual_points: np.ndarray,
    us_corner_points: np.ndarray,
    us_interp_points: np.ndarray | None = None,
    alpha_soft=0.4,
    margin=0.03,
    w_tof=1.0,
    w_ultra=0.6,
    w_corner=0.25,
    w_interp=0.35,
    # If your XYZ columns are not (0,1,2), adjust per-set here:
    tof_xyz_cols=(0,1,2),
    us_actual_xyz_cols=(0,1,2),
    us_corner_xyz_cols=(0,1,2),
    us_interp_xyz_cols=(0,1,2),
):
    """
    Fuse ToF + Ultrasonic + Corner priors, optionally including interpolated wall fills.
    Coerces each set to XYZ with ensure_xyz() so concatenation is safe.

    Returns:
      fused_points: (M,3)
      fused_weights: (M,)
      corners_xy: (K,2) compact CCW polygon (for overlays)
      tof_clipped: (N_tof,3) ToF after soft-clip (for subplot)
    """
    # Coerce to XYZ (handles Nx6 etc.)
    tof_xyz       = ensure_xyz(tof_points,       xyz_cols=tof_xyz_cols)
    us_actual_xyz = ensure_xyz(us_actual_points, xyz_cols=us_actual_xyz_cols)
    us_corner_xyz = ensure_xyz(us_corner_points, xyz_cols=us_corner_xyz_cols)
    us_interp_xyz = ensure_xyz(us_interp_points, xyz_cols=us_interp_xyz_cols) if (us_interp_points is not None and len(us_interp_points) > 0) else None

    # Build compact convex polygon from many corners (fast)
    corners_xy = build_room_polygon(us_corner_xyz[:, :2], max_vertices=24)

    # Soft-clip ToF in XY (keep Z intact)
    tof_clipped = tof_xyz.copy()
    tof_clipped[:, :2] = soft_clip_to_convex_polygon(
        tof_xyz[:, :2], corners_xy, alpha=alpha_soft, margin=margin
    )

    # Assemble fused cloud + weights
    clouds = [tof_clipped, us_actual_xyz, us_corner_xyz]
    weights = [
        np.full(len(tof_clipped), w_tof, dtype=float),
        np.full(len(us_actual_xyz), w_ultra, dtype=float),
        np.full(len(us_corner_xyz), w_corner, dtype=float),
    ]
    if us_interp_xyz is not None and len(us_interp_xyz) > 0:
        clouds.append(us_interp_xyz)
        weights.append(np.full(len(us_interp_xyz), w_interp, dtype=float))

    fused_points  = np.vstack(clouds)
    fused_weights = np.concatenate(weights)
    return fused_points, fused_weights, corners_xy, tof_clipped

# =============================================================================
# Visualization
# =============================================================================

def _plot_room_polygon(ax, poly_xy, **kwargs):
    """Draw the room polygon as a closed outline."""
    poly = np.asarray(poly_xy)
    loop = np.vstack([poly, poly[0]])
    ax.plot(loop[:, 0], loop[:, 1], **kwargs)

def visualize_stages(
    tof_raw,
    us_actual,
    us_corners,
    us_interp,
    corners_xy,
    tof_clipped,
    fused_points_interp_on,
    fused_weights_interp_on,
    fused_points_interp_off,
    fused_weights_interp_off,
    traj_positions=None,
    title_suffix=""
):
    """
    2×2 subplot:
      (1,1) Inputs           (ToF raw + ultrasonic actual + corners + optional traj)
      (1,2) ToF clipped      (after corner-aware soft-clip)
      (2,1) Fused (interp ON)  colored by weights
      (2,2) Fused (interp OFF) colored by weights
    """
    # Downsample ONLY for plotting so Matplotlib stays fast
    tof_raw_plot   = _downsample(tof_raw,   20000)
    us_interp_plot = _downsample(us_interp, 30000) if us_interp is not None else None
    fused_on_plot  = _downsample(fused_points_interp_on, 40000)
    w_on_plot      = _downsample(fused_weights_interp_on, 40000)
    fused_off_plot = _downsample(fused_points_interp_off, 40000)
    w_off_plot     = _downsample(fused_weights_interp_off, 40000)

    fig, axes = plt.subplots(2, 2, figsize=(12, 10), constrained_layout=True)
    ax11, ax12, ax21, ax22 = axes.ravel()

    # --- (1,1) Inputs ---
    ax = ax11
    ax.set_title("Stage 0 • Inputs (Top-down XY)")
    if tof_raw_plot is not None and len(tof_raw_plot):
        ax.scatter(tof_raw_plot[:, 0], tof_raw_plot[:, 1], s=1, alpha=0.35, label="ToF raw")
    if us_actual is not None and len(us_actual):
        ax.scatter(us_actual[:, 0], us_actual[:, 1], s=30, marker="x", label="Ultrasonic actual")
    if us_corners is not None and len(us_corners):
        ax.scatter(us_corners[:, 0], us_corners[:, 1], s=40, marker="s", label="Extrapolated corners")
    if us_interp_plot is not None and len(us_interp_plot) > 0:
        ax.scatter(us_interp_plot[:, 0], us_interp_plot[:, 1], s=8, alpha=0.7, label="Ultrasonic interp (optional)")
    _plot_room_polygon(ax, corners_xy, linewidth=1.0)
    if traj_positions is not None and len(traj_positions):
        ax.plot(traj_positions[:, 0], traj_positions[:, 1], linewidth=1.0, label="Trajectory")
    ax.set_aspect("equal", "box")
    ax.legend(loc="best")

    # --- (1,2) Clipped ToF ---
    ax = ax12
    ax.set_title("Stage 1 • ToF after corner-aware soft-clip")
    if tof_clipped is not None and len(tof_clipped):
        tc_plot = _downsample(tof_clipped, 20000)
        ax.scatter(tc_plot[:, 0], tc_plot[:, 1], s=1, alpha=0.6, label="ToF clipped")
    _plot_room_polygon(ax, corners_xy, linewidth=1.0)
    ax.set_aspect("equal", "box")
    ax.legend(loc="best")

    # --- (2,1) Fused (interp ON) ---
    ax = ax21
    ax.set_title("Stage 2A • Fused cloud (interpolation ON)")
    if fused_on_plot is not None and len(fused_on_plot):
        sc = ax.scatter(
            fused_on_plot[:, 0],
            fused_on_plot[:, 1],
            s=2,
            c=w_on_plot,
            cmap="viridis",
            alpha=0.9,
            label="Fused points"
        )
        cb = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
        cb.set_label("Per-point weight")
    _plot_room_polygon(ax, corners_xy, linewidth=1.0)
    ax.set_aspect("equal", "box")
    ax.legend(loc="best")

    # --- (2,2) Fused (interp OFF) ---
    ax = ax22
    ax.set_title("Stage 2B • Fused cloud (interpolation OFF)")
    if fused_off_plot is not None and len(fused_off_plot):
        sc = ax.scatter(
            fused_off_plot[:, 0],
            fused_off_plot[:, 1],
            s=2,
            c=w_off_plot,
            cmap="viridis",
            alpha=0.9,
            label="Fused points"
        )
        cb = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
        cb.set_label("Per-point weight")
    _plot_room_polygon(ax, corners_xy, linewidth=1.0)
    ax.set_aspect("equal", "box")
    ax.legend(loc="best")

    if title_suffix:
        fig.suptitle(title_suffix, fontsize=12)

    plt.show()

# =============================================================================
# Main Execution
# =============================================================================

def cut_data(data_times_csv, us_input_csv, us_output_csv):
    """Your existing cropper. Placeholder to keep this file stand-alone."""
    pass

def main():
    t0 = time.time()

    # 0) Paths & data selection
    data_name = "22_09_25_MILC/7_lyco_lab/"
    base_path = "../optical_flow_method_data/combined_samples/" + data_name

    data_times = base_path + "data_times.csv"
    tof_input_cropped = base_path + "download_tof_cropped.csv"
    us_input_path = base_path + "fake_ultrasonic.csv"
    us_input_cropped = base_path + "us_cropped.csv"

    # 1) Crop ultrasonic (if needed)
    cut_data(data_times, us_input_path, us_input_cropped)

    # 2) Generate maps
    print("\n=== Stage 1: Generate ToF and Ultrasonic Maps ===")
    tof_points, traj_positions = tof_map(tof_input_cropped)
    us_interp_points, us_actual_points, us_corner_points = us_map(us_input_cropped)

    print(f"ToF points: {len(tof_points)} | US actual: {len(us_actual_points)} | "
          f"US corners: {len(us_corner_points)} | US interp: {len(us_interp_points)}")
    t1 = time.time()
    print(f"Stage 1 time: {t1 - t0:.2f}s")

    # 3) Corner-aware fusion (two modes to compare)
    print("\n=== Stage 2: Corner-aware fusion ===")
    # Mode A: with interpolation (soft prior)
    t2a0 = time.time()
    fused_on, w_on, room_poly_xy, tof_clipped = fuse_ultra_tof(
        tof_points=tof_points,
        us_actual_points=us_actual_points,
        us_corner_points=us_corner_points,
        us_interp_points=us_interp_points,
        alpha_soft=0.4,   # 0.2–0.5 tighter pull; 0 = hard snap
        margin=0.03,      # 3 cm erosion
        w_tof=1.0,
        w_ultra=0.6,
        w_corner=0.25,
        w_interp=0.35,
        # If your XYZ are not cols (0,1,2), set here:
        tof_xyz_cols=(0,1,2),
        us_actual_xyz_cols=(0,1,2),
        us_corner_xyz_cols=(0,1,2),
        us_interp_xyz_cols=(0,1,2),
    )
    t2a1 = time.time()
    print(f"Fusion (interp ON): {t2a1 - t2a0:.2f}s | points: {len(fused_on)}")

    # Mode B: without interpolation (truth-preserving)
    t2b0 = time.time()
    fused_off, w_off, _, _ = fuse_ultra_tof(
        tof_points=tof_points,
        us_actual_points=us_actual_points,
        us_corner_points=us_corner_points,
        us_interp_points=None,
        alpha_soft=0.4,
        margin=0.03,
        w_tof=1.0,
        w_ultra=0.6,
        w_corner=0.25,
        w_interp=0.35,
        tof_xyz_cols=(0,1,2),
        us_actual_xyz_cols=(0,1,2),
        us_corner_xyz_cols=(0,1,2),
        us_interp_xyz_cols=(0,1,2),
    )
    t2b1 = time.time()
    print(f"Fusion (interp OFF): {t2b1 - t2b0:.2f}s | points: {len(fused_off)}")

    # 4) Visualize stages
    tv0 = time.time()
    visualize_stages(
        tof_raw=ensure_xyz(tof_points),  # ensure plotting sees XYZ
        us_actual=ensure_xyz(us_actual_points),
        us_corners=ensure_xyz(us_corner_points),
        us_interp=ensure_xyz(us_interp_points) if (us_interp_points is not None and len(us_interp_points) > 0) else None,
        corners_xy=room_poly_xy,
        tof_clipped=tof_clipped,
        fused_points_interp_on=fused_on,
        fused_weights_interp_on=w_on,
        fused_points_interp_off=fused_off,
        fused_weights_interp_off=w_off,
        traj_positions=traj_positions,
        title_suffix="Ultrasonic + ToF Fusion (Corner-aware soft-clamp)"
    )
    tv1 = time.time()
    print(f"Visualization time: {tv1 - tv0:.2f}s")

    print(f"\nTotal time: {time.time() - t0:.2f}s")

# Entry point
if __name__ == "__main__":
    main()
