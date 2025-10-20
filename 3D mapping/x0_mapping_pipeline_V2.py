"""
x0_mapping_pipeline_V2.py
-----------------------
Version 2 of the mapping pipeline for the ELEC5550 Design Project (2025),
"Indoor 3D Mapping Drone".

This version constrains ToF points based on the direction and range of
ultrasonic sensor measurements. If a ToF point extends beyond the
ultrasonic-detected surface in that direction, it is pulled inward
toward the ultrasonic boundary.

Concept:
    - Ultrasonic readings define the nearest real surfaces in each direction.
    - ToF readings are dense but can overshoot (fan-out effect).
    - Any ToF point that lies *past* the ultrasonic boundary (same direction)
      is moved halfway (50%) toward that surface.

Color Legend:
    Black  = ToF (original + constrained)
    Dark Blue = Ultrasonic points (actual + corners)
    Red    = Drone trajectory
"""

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import time
from tof_map_V2 import main as tof_map
from us_map_V3 import main as us_map


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


# === Stage 2: Directional constraint based on ultrasonic readings ===
def directional_constrain_tof(tof_points, us_actual_points, us_corner_points,
                              pull_ratio=0.5, direction_threshold=0.95):
    """
    Pull ToF points inward if they extend beyond the ultrasonic boundary
    in the same direction of measurement.
    """
    print("\n=== Stage 2: Directionally Constraining ToF Points ===")

    tof_xyz = np.asarray(tof_points)[:, :3]
    us_actual_xyz = np.asarray(us_actual_points)[:, :3]
    us_corner_xyz = np.asarray(us_corner_points)[:, :3]

    if len(us_actual_xyz) == 0:
        raise ValueError("No ultrasonic actual points provided.")
    if len(us_corner_xyz) == 0:
        print("⚠️ No corner points provided, using only actual ultrasonic data.")
        us_combined = us_actual_xyz
    else:
        us_combined = np.vstack([us_actual_xyz, us_corner_xyz])

    us_pcd = o3d.geometry.PointCloud()
    us_pcd.points = o3d.utility.Vector3dVector(us_combined)
    us_tree = o3d.geometry.KDTreeFlann(us_pcd)

    adjusted_points = np.copy(tof_xyz)
    pulled_count = 0

    for i, tof_pt in enumerate(tof_xyz):
        tof_vec = np.asarray(tof_pt, dtype=np.float64)

        try:
            k, idx, _ = us_tree.search_knn_vector_3d(tof_vec, 1)
        except Exception:
            continue
        if k == 0:
            continue

        nearest_us = us_combined[idx[0]]
        tof_dir = tof_vec / (np.linalg.norm(tof_vec) + 1e-6)
        us_dir = nearest_us / (np.linalg.norm(nearest_us) + 1e-6)

        if np.dot(tof_dir, us_dir) > direction_threshold:
            d_tof = np.linalg.norm(tof_vec)
            d_us = np.linalg.norm(nearest_us)
            if d_tof > d_us:
                overshoot = d_tof - d_us
                adjusted_points[i] = tof_vec - pull_ratio * overshoot * tof_dir
                pulled_count += 1

    print(f"✅ Adjusted {pulled_count} ToF points exceeding ultrasonic boundaries.")

    pcd_adj = o3d.geometry.PointCloud()
    pcd_adj.points = o3d.utility.Vector3dVector(adjusted_points)
    pcd_adj.paint_uniform_color([0.0, 0.0, 0.0])  # black (adjusted)
    return pcd_adj, adjusted_points


# === Stage 3a: Open3D overlay comparison ===
def visualize_directional_constraint(tof_points, adjusted_points, us_actual_points, us_corner_points):
    geoms = []

    # Original ToF (black)
    pcd_tof = o3d.geometry.PointCloud()
    pcd_tof.points = o3d.utility.Vector3dVector(tof_points[:, :3])
    pcd_tof.paint_uniform_color([98/255, 200/255, 211/255])
    geoms.append(pcd_tof)

    # Adjusted ToF (black)
    pcd_adj = o3d.geometry.PointCloud()
    pcd_adj.points = o3d.utility.Vector3dVector(adjusted_points[:, :3])
    pcd_adj.paint_uniform_color([0/255, 113/255, 145/255])
    geoms.append(pcd_adj)

    # Ultrasonic actual + corners (dark blue)
    if us_actual_points is not None and len(us_actual_points) > 0:
        pc_us_actual = o3d.geometry.PointCloud()
        pc_us_actual.points = o3d.utility.Vector3dVector(us_actual_points[:, :3])
        pc_us_actual.paint_uniform_color([1.0, 0.0, 0.0])
        geoms.append(pc_us_actual)
    if us_corner_points is not None and len(us_corner_points) > 0:
        pc_us_corner = o3d.geometry.PointCloud()
        pc_us_corner.points = o3d.utility.Vector3dVector(us_corner_points[:, :3])
        pc_us_corner.paint_uniform_color([1.0, 0.0, 0.0])
        geoms.append(pc_us_corner)

    print("\n✅ Visualizing ToF constraint (overlay)")
    o3d.visualization.draw_geometries(geoms, window_name="Directional Constraint Comparison")


# === Stage 3b: Matplotlib side-by-side comparison ===
def visualize_directional_comparison_matplotlib(tof_points, adjusted_points, us_actual_points, us_corner_points):
    print("\n📊 Visualizing ToF constraint (side-by-side subplots)")
    tof_xyz = np.asarray(tof_points)[:, :3]
    adj_xyz = np.asarray(adjusted_points)[:, :3]
    us_actual_xyz = np.asarray(us_actual_points)[:, :3]
    us_corner_xyz = np.asarray(us_corner_points)[:, :3]

    fig = plt.figure(figsize=(14, 7))
    ax1 = fig.add_subplot(121, projection="3d")
    ax2 = fig.add_subplot(122, projection="3d")

    # Before constraint (black)
    ax1.scatter(tof_xyz[:, 0], tof_xyz[:, 1], tof_xyz[:, 2],
                c=(98/255, 200/255, 211/255), s=1, label="Original ToF")
    if len(us_actual_xyz):
        ax1.scatter(us_actual_xyz[:, 0], us_actual_xyz[:, 1], us_actual_xyz[:, 2],
                    c=(1.0, 0.0, 0.0), s=4, label="Ultrasonic Actual")
    if len(us_corner_xyz):
        ax1.scatter(us_corner_xyz[:, 0], us_corner_xyz[:, 1], us_corner_xyz[:, 2],
                    c=(1.0, 0.0, 0.0), s=4, label="Ultrasonic Corners")
    ax1.set_title("Before Constraint (Original ToF)")
    ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)"); ax1.set_zlabel("Z (m)")
    ax1.legend(loc="upper right")
    ax1.set_box_aspect([1, 1, 1])

    # After constraint (black)
    ax2.scatter(adj_xyz[:, 0], adj_xyz[:, 1], adj_xyz[:, 2],
                c=(0/255, 113/255, 145/255), s=1, label="Constrained ToF")
    if len(us_actual_xyz):
        ax2.scatter(us_actual_xyz[:, 0], us_actual_xyz[:, 1], us_actual_xyz[:, 2],
                    c=(1.0, 0.0, 0.0), s=4, label="Ultrasonic Actual")
    if len(us_corner_xyz):
        ax2.scatter(us_corner_xyz[:, 0], us_corner_xyz[:, 1], us_corner_xyz[:, 2],
                    c=(1.0, 0.0, 0.0), s=4, label="Ultrasonic Corners")
    ax2.set_title("After Constraint (Adjusted ToF)")
    ax2.set_xlabel("X (m)"); ax2.set_ylabel("Y (m)"); ax2.set_zlabel("Z (m)")
    ax2.legend(loc="upper right")
    ax2.set_box_aspect([1, 1, 1])

    plt.tight_layout()
    plt.show()


# === Stage 4: Final combined visualization ===
def visualize_fused_map(adjusted_points, us_actual_points, us_corner_points, traj_positions):
    geoms = []

    # Adjusted ToF (black)
    pcd_adj = o3d.geometry.PointCloud()
    pcd_adj.points = o3d.utility.Vector3dVector(adjusted_points[:, :3])
    pcd_adj.paint_uniform_color([0/255, 113/255, 145/255])
    geoms.append(pcd_adj)

    # Ultrasonic (dark blue)
    if us_actual_points is not None and len(us_actual_points) > 0:
        pc_us_actual = o3d.geometry.PointCloud()
        pc_us_actual.points = o3d.utility.Vector3dVector(us_actual_points[:, :3])
        pc_us_actual.paint_uniform_color([1.0, 0.0, 0.0])
        geoms.append(pc_us_actual)
    if us_corner_points is not None and len(us_corner_points) > 0:
        pc_us_corner = o3d.geometry.PointCloud()
        pc_us_corner.points = o3d.utility.Vector3dVector(us_corner_points[:, :3])
        pc_us_corner.paint_uniform_color([1.0, 0.0, 0.0])
        geoms.append(pc_us_corner)

    # Drone trajectory (red)
    if traj_positions is not None and len(traj_positions) > 1:
        traj = o3d.geometry.LineSet()
        traj.points = o3d.utility.Vector3dVector(traj_positions)
        traj.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(traj_positions) - 1)])
        traj.colors = o3d.utility.Vector3dVector([[1.0, 0.176, 0.667] for _ in range(len(traj_positions) - 1)])
        geoms.append(traj)
        marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        marker.translate(traj_positions[-1])
        marker.paint_uniform_color([1.0, 0.176, 0.667])
        geoms.append(marker)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    geoms.append(axis)

    o3d.visualization.draw_geometries(geoms, window_name="Final Fused Constrained Map")


# === Main Execution ===
def main():
    t0 = time.time()

    data_name = "22_09_25_MILC/7_lyco_lab/"
    base_path = "../optical_flow_method_data/combined_samples/" + data_name

    data_times = base_path + "data_times.csv"
    tof_input_cropped = base_path + "download_tof_cropped.csv"
    us_input_path = base_path + "fake_ultrasonic.csv"
    us_input_cropped = base_path + "us_cropped.csv"

    # 1️⃣ Crop ultrasonic data
    cut_data(data_times, us_input_path, us_input_cropped)

    # 2️⃣ Generate maps
    print("\n=== Stage 1: Generate ToF and Ultrasonic Maps ===")
    tof_points, traj_positions = tof_map(tof_input_cropped)
    us_interp_points, us_actual_points, us_corner_points = us_map(us_input_cropped)

    # 3️⃣ Apply directional constraint
    adjusted_pcd, adjusted_points = directional_constrain_tof(
        tof_points, us_actual_points, us_corner_points,
        pull_ratio=0.5, direction_threshold=0.95
    )

    # 4️⃣ Open3D overlay comparison
    visualize_directional_constraint(tof_points, adjusted_points, us_actual_points, us_corner_points)

    # 5️⃣ Matplotlib side-by-side comparison
    visualize_directional_comparison_matplotlib(tof_points, adjusted_points, us_actual_points, us_corner_points)

    # 6️⃣ Final fused map
    visualize_fused_map(adjusted_points, us_actual_points, us_corner_points, traj_positions)

    print(f"\nPipeline complete in {time.time() - t0:.2f}s")

if __name__ == "__main__":
    main()
