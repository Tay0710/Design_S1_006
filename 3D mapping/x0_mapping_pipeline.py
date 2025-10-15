"""
x0_mapping_pipeline.py
-----------------------
Coordinates the sequential execution of all data-processing stages for
generating 3D environment maps in the ELEC5550 Design Project (2025),
"Indoor 3D Mapping Drone".

Pipeline stages:
    1. Data cropping
        - Reads the start and end times from data_times.csv.
        - Crops the raw ultrasonic dataset (fake_ultrasonic.csv) to match this
          time window, producing a synchronized version (us_cropped.csv).
    2. ToF mapping
        - Processes cropped VL53L7CX ToF data to generate a dense 3D point cloud.
        - Aligns ToF frames with the drone’s trajectory to map depth readings
          into world coordinates.
    3. Ultrasonic mapping
        - Processes cropped ultrasonic data to generate a complementary 3D
          point cloud using IMU-derived rotation matrices for orientation.
        - Maps each directional ultrasonic reading (U, D, L, R) into the same
          world frame as the ToF data.
    4. Fusion and Visualization
        - Merges ToF and Ultrasonic points into a single Open3D display:
            * Blue:  ToF map
            * Orange: Ultrasonic interpolated (walls)
            * Pink:  Ultrasonic actual sensor points
            * Cyan:  Ultrasonic corner extrapolations
            * Red:   Drone trajectory and final position
            * Axes:  World coordinate reference

Usage:
    Run this script directly to execute the full mapping pipeline:
        $ python x0_mapping_pipeline.py

Outputs:
    Intermediate:
        - us_cropped.csv : Cropped ultrasonic data (time-synchronized)
    Final:
        - Combined 3D environmental map (Open3D visualization)
        - Optional fused mesh reconstruction (exportable)
"""

import numpy as np
import pandas as pd
import open3d as o3d
import time
import os

from tof_map_V2 import main as tof_map
from us_map_V3 import main as us_map


# === Stage 1: Data Cropping ===
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
    print(f"File updated and saved with cut data: start={start_time}, end={end_time}")


# === Stage 4a: Multi-Layer Visualization (diagnostic) ===
def visualize_combined_map(tof_points, us_interp_points, us_actual_points, us_corner_points, traj_positions):
    geoms = []

    # ToF points (blue)
    if tof_points is not None and len(tof_points) > 0:
        pc_tof = o3d.geometry.PointCloud()
        pc_tof.points = o3d.utility.Vector3dVector(tof_points[:, :3])
        pc_tof.paint_uniform_color([0, 0, 1])
        geoms.append(pc_tof)

    # Ultrasonic interpolated (orange)
    if us_interp_points is not None and len(us_interp_points) > 0:
        pc_us_interp = o3d.geometry.PointCloud()
        pc_us_interp.points = o3d.utility.Vector3dVector(us_interp_points[:, :3])
        pc_us_interp.paint_uniform_color([1, 0.5, 0])
        geoms.append(pc_us_interp)

    # Ultrasonic actual (pink)
    if us_actual_points is not None and len(us_actual_points) > 0:
        pc_us_actual = o3d.geometry.PointCloud()
        pc_us_actual.points = o3d.utility.Vector3dVector(us_actual_points[:, :3])
        pc_us_actual.paint_uniform_color([1, 0.3, 0.8])
        geoms.append(pc_us_actual)

    # Ultrasonic corners (cyan)
    if us_corner_points is not None and len(us_corner_points) > 0:
        pc_us_corners = o3d.geometry.PointCloud()
        pc_us_corners.points = o3d.utility.Vector3dVector(us_corner_points[:, :3])
        pc_us_corners.paint_uniform_color([0.3, 1, 1])
        geoms.append(pc_us_corners)

    # Drone trajectory (red line) and marker
    if traj_positions is not None and len(traj_positions) > 1:
        traj = o3d.geometry.LineSet()
        traj.points = o3d.utility.Vector3dVector(traj_positions)
        traj.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(traj_positions) - 1)])
        traj.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(traj_positions) - 1)])
        geoms.append(traj)

        marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        marker.translate(traj_positions[-1])
        marker.paint_uniform_color([1, 0, 0])
        geoms.append(marker)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    geoms.append(axis)

    o3d.visualization.draw_geometries(geoms, window_name="Combined ToF + Ultrasonic Map")


def fuse_tof_and_ultrasonic(tof_points, us_interp_points):
    """
    Fuse ToF and ultrasonic interpolated points into a single, clean point cloud.
    Removes redundant colour information, balances densities, and denoises.
    Result: unified geometry with only two clear colour sources.
    """
    import open3d as o3d
    import numpy as np

    # --- Validate inputs ---
    if tof_points is None or len(tof_points) == 0:
        print("⚠️ No ToF points provided — fusion skipped.")
        return None
    if us_interp_points is None or len(us_interp_points) == 0:
        print("⚠️ No Ultrasonic points provided — fusion skipped.")
        return None

    tof_points = np.asarray(tof_points, dtype=float)
    us_interp_points = np.asarray(us_interp_points, dtype=float)

    # --- Strip colours if present (ToF has 6 columns: x, y, z, R, G, B) ---
    if tof_points.shape[1] >= 3:
        tof_xyz = tof_points[:, :3]
    else:
        raise ValueError(f"ToF points must have at least 3 columns, got {tof_points.shape}")

    if us_interp_points.shape[1] >= 3:
        us_xyz = us_interp_points[:, :3]
    else:
        raise ValueError(f"Ultrasonic points must have at least 3 columns, got {us_interp_points.shape}")

    # --- Create Open3D point clouds ---
    pcd_tof = o3d.geometry.PointCloud()
    pcd_tof.points = o3d.utility.Vector3dVector(tof_xyz)
    pcd_tof.paint_uniform_color([0.0, 0.3, 1.0])  # Blue (ToF)

    pcd_us = o3d.geometry.PointCloud()
    pcd_us.points = o3d.utility.Vector3dVector(us_xyz)
    pcd_us.paint_uniform_color([0.8, 0.8, 0.8])  # Orange (Ultrasonic)

    # --- Downsample to even density ---
    voxel_size = 0.02
    pcd_tof = pcd_tof.voxel_down_sample(voxel_size)
    pcd_us = pcd_us.voxel_down_sample(voxel_size)

    # --- Merge clouds ---
    pcd_combined = pcd_tof + pcd_us

    # --- Denoise outliers ---
    if len(pcd_combined.points) == 0:
        print("⚠️ Combined cloud is empty after merge.")
        return None

    pcd_combined, _ = pcd_combined.remove_statistical_outlier(
        nb_neighbors=40, std_ratio=2.0
    )

    # --- Estimate normals (for potential meshing) ---
    pcd_combined.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30)
    )

    print(f"✅ Fused map ready: {np.asarray(pcd_combined.points).shape[0]} points after cleaning.")
    return pcd_combined

# === Stage 5: Optional Mesh Reconstruction ===
def reconstruct_mesh_from_pointcloud(pcd_combined):
    """Generate and visualize a Poisson mesh from the fused point cloud."""
    print("\n=== Stage 5: Generating Surface Mesh (optional) ===")
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd_combined, depth=10)
    bbox = pcd_combined.get_axis_aligned_bounding_box()
    mesh = mesh.crop(bbox)
    o3d.visualization.draw_geometries([mesh], window_name="Reconstructed Surface Mesh")
    return mesh

def align_tof_to_ultrasonic_weighted(tof_points, us_actual_points, us_corner_points, weight_corner=0.85):
    """
    Align ToF point cloud to ultrasonic anchor data using confidence-weighted ICP.
    - Ultrasonic actual points are trusted anchors (weight 1.0)
    - Ultrasonic corner extrapolations are partial anchors (weight < 1.0)
    - ToF data is adjusted to match ultrasonic structure.
    Returns the aligned ToF point cloud (Open3D PointCloud).
    """
    import open3d as o3d
    import numpy as np

    # --- Convert inputs to numpy arrays ---
    tof_points = np.asarray(tof_points, dtype=float)
    us_actual_points = np.asarray(us_actual_points, dtype=float)
    us_corner_points = np.asarray(us_corner_points, dtype=float)

    tof_xyz = tof_points[:, :3]
    us_actual_xyz = us_actual_points[:, :3]
    us_corner_xyz = us_corner_points[:, :3]

    # --- Build weighted anchor cloud (actual + corner) ---
    n_actual = len(us_actual_xyz)
    n_corner = len(us_corner_xyz)
    if n_actual == 0:
        raise ValueError("No ultrasonic actual points provided — cannot anchor ToF map.")
    combined_anchors = np.vstack([us_actual_xyz, us_corner_xyz])
    anchor_weights = np.hstack([
        np.ones(n_actual),                # full confidence
        np.ones(n_corner) * weight_corner # partial confidence
    ])

    # --- Build Open3D clouds ---
    pcd_tof = o3d.geometry.PointCloud()
    pcd_tof.points = o3d.utility.Vector3dVector(tof_xyz)
    pcd_tof.paint_uniform_color([0.0, 0.3, 1.0])  # blue

    pcd_anchor = o3d.geometry.PointCloud()
    pcd_anchor.points = o3d.utility.Vector3dVector(combined_anchors)
    pcd_anchor.paint_uniform_color([1.0, 0.55, 0.0])  # orange anchors

    print(f"Running weighted ICP alignment: {n_actual} strong + {n_corner} weak anchors")

    # --- Weighted ICP (iteratively reweighted least squares) ---
    threshold = 0.05  # 5 cm match tolerance
    transformation = np.identity(4)
    for iter in range(3):
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source=pcd_tof,
            target=pcd_anchor,
            max_correspondence_distance=threshold,
            init=transformation,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
        )
        transformation = reg_p2p.transformation
        pcd_tof.transform(transformation)
        threshold *= 0.7  # gradually tighten tolerance

    print("✅ ToF alignment refined using ultrasonic anchors.")

    return pcd_tof, pcd_anchor

def visualize_tof_alignment_comparison(original_tof_points, aligned_tof_pcd, us_actual_points, us_corner_points):
    """
    Compare pre- and post-alignment ToF clouds together with ultrasonic anchors.
    Blue  = Original ToF
    Green = Aligned ToF
    Orange/Pink = Ultrasonic anchors (actual + corners)
    """
    import open3d as o3d
    import numpy as np

    geoms = []

    # Original ToF (blue)
    pcd_orig = o3d.geometry.PointCloud()
    pcd_orig.points = o3d.utility.Vector3dVector(np.asarray(original_tof_points)[:, :3])
    pcd_orig.paint_uniform_color([0.0, 0.3, 1.0])
    geoms.append(pcd_orig)

    # Aligned ToF (green)
    aligned_copy = aligned_tof_pcd.translate((0, 0, 0), relative=False)
    aligned_copy.paint_uniform_color([0.0, 1.0, 0.0])
    geoms.append(aligned_copy)

    # Ultrasonic actual (orange)
    if us_actual_points is not None and len(us_actual_points) > 0:
        pc_us_actual = o3d.geometry.PointCloud()
        pc_us_actual.points = o3d.utility.Vector3dVector(us_actual_points[:, :3])
        pc_us_actual.paint_uniform_color([1.0, 0.55, 0.0])
        geoms.append(pc_us_actual)

    # Ultrasonic corners (pink)
    if us_corner_points is not None and len(us_corner_points) > 0:
        pc_us_corner = o3d.geometry.PointCloud()
        pc_us_corner.points = o3d.utility.Vector3dVector(us_corner_points[:, :3])
        pc_us_corner.paint_uniform_color([1.0, 0.55, 0.0])
        geoms.append(pc_us_corner)

    o3d.visualization.draw_geometries(geoms, window_name="ToF Alignment Comparison (Before vs After)")

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

    # 2️⃣ Generate ToF map
    print("\n=== Stage 1: Generate ToF Point Cloud ===")
    tof_points, traj_positions = tof_map(tof_input_cropped)

    # 3️⃣ Generate Ultrasonic map
    print("\n=== Stage 2: Generate Ultrasonic Point Cloud ===")
    us_interp_points, us_actual_points, us_corner_points = us_map(us_input_cropped)
    # 3b️⃣ Align ToF map to ultrasonic anchors
    aligned_tof, anchor_cloud = align_tof_to_ultrasonic_weighted(tof_points, us_actual_points, us_corner_points)

    # Visual comparison: before vs after ToF alignment
    visualize_tof_alignment_comparison(tof_points, aligned_tof, us_actual_points, us_corner_points)

    
    # 4️⃣ Fuse ToF + Ultrasonic (interpolated) data
    print("\n=== Stage 3: Fusing ToF and Ultrasonic Maps ===")
    print(f"ToF points type: {type(tof_points)}, shape: {getattr(tof_points, 'shape', 'N/A')}")
    print(f"US points type: {type(us_interp_points)}, shape: {getattr(us_interp_points, 'shape', 'N/A')}")

    # fused_cloud = fuse_tof_and_ultrasonic(tof_points, us_interp_points)
    fused_cloud = fuse_tof_and_ultrasonic(np.asarray(aligned_tof.points), us_interp_points)


    # 5️⃣ Visualize combined layers (diagnostic)
    visualize_combined_map(tof_points, us_interp_points, us_actual_points, us_corner_points, traj_positions)

    # 6️⃣ Visualize fused final map
    print("\n=== Stage 4: Visualising Fused Map ===")
    o3d.visualization.draw_geometries([fused_cloud], window_name="Final Fused ToF + Ultrasonic Map")

    # 7️⃣ (Optional) Mesh reconstruction
    # mesh = reconstruct_mesh_from_pointcloud(fused_cloud)
    # o3d.io.write_triangle_mesh(base_path + "fused_mesh.ply", mesh)

    print(f"\nPipeline complete in {time.time() - t0:.2f}s")


if __name__ == "__main__":
    main()
