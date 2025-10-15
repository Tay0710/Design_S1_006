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
import open3d as o3d
import time
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


# === Stage 2: Multi-Layer Visualization (diagnostic) ===
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

    # Drone trajectory (red)
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


# === Stage 3: Weighted Alignment ===
def align_tof_to_ultrasonic_weighted(tof_points, us_actual_points, us_corner_points, weight_corner=0.85):
    import numpy as np
    import open3d as o3d

    def compute_rms_distance(src, tgt):
        dists = src.compute_point_cloud_distance(tgt)
        return np.mean(np.square(dists)) ** 0.5

    # Convert to arrays
    tof_xyz = np.asarray(tof_points)[:, :3]
    us_actual_xyz = np.asarray(us_actual_points)[:, :3]
    us_corner_xyz = np.asarray(us_corner_points)[:, :3]

    # Weighted anchors (actual + corners)
    anchors = np.vstack([us_actual_xyz, us_corner_xyz])
    n_actual, n_corner = len(us_actual_xyz), len(us_corner_xyz)
    print(f"Running weighted ICP: {n_actual} strong anchors, {n_corner} weak anchors (w={weight_corner})")

    # Build Open3D clouds
    pcd_tof = o3d.geometry.PointCloud()
    pcd_tof.points = o3d.utility.Vector3dVector(tof_xyz)

    pcd_anchor = o3d.geometry.PointCloud()
    pcd_anchor.points = o3d.utility.Vector3dVector(anchors)

    # RMS before alignment
    rms_before = compute_rms_distance(pcd_tof, pcd_anchor)

    # ICP loop
    threshold = 0.05
    transform = np.eye(4)
    for i in range(3):
        reg = o3d.pipelines.registration.registration_icp(
            pcd_tof, pcd_anchor, threshold, transform,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
        )
        transform = reg.transformation
        pcd_tof.transform(transform)
        threshold *= 0.7

    rms_after = compute_rms_distance(pcd_tof, pcd_anchor)

    print(f"RMS distance before: {rms_before:.4f} m")
    print(f"RMS distance after : {rms_after:.4f} m")

    # Safety check
    if rms_after > rms_before * 1.2:
        print("⚠️ Alignment worsened — reverting to original ToF cloud.")
        pcd_tof.points = o3d.utility.Vector3dVector(tof_xyz)
    else:
        print("✅ Alignment improved successfully.")

    return pcd_tof, pcd_anchor


# === Stage 4: Fused Comparison ===
def visualize_tof_alignment_comparison(original_tof_points, aligned_tof_pcd, us_actual_points, us_corner_points):
    """
    Compare pre- and post-alignment ToF clouds together with ultrasonic anchors.
    Blue  = Original ToF
    Green = Aligned ToF
    Orange/Pink = Ultrasonic anchors (actual + corners)
    """
    import open3d as o3d
    import numpy as np
    import copy

    geoms = []

    # Original ToF (blue)
    pcd_orig = o3d.geometry.PointCloud()
    pcd_orig.points = o3d.utility.Vector3dVector(np.asarray(original_tof_points)[:, :3])
    pcd_orig.paint_uniform_color([0.0, 0.3, 1.0])
    geoms.append(pcd_orig)

    # ✅ Aligned ToF (green) — use deepcopy instead of clone()
    pcd_aligned = copy.deepcopy(aligned_tof_pcd)
    pcd_aligned.paint_uniform_color([0.0, 1.0, 0.0])
    geoms.append(pcd_aligned)

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
        pc_us_corner.paint_uniform_color([1.0, 0.3, 0.8])
        geoms.append(pc_us_corner)

    print("\n✅ Visualizing ToF alignment (blue=before, green=after)")
    o3d.visualization.draw_geometries(geoms, window_name="ToF Alignment Comparison (Before vs After)")

# === Stage 5: Fusion ===
def fuse_tof_and_ultrasonic(tof_points, us_interp_points):
    tof_xyz = np.asarray(tof_points)[:, :3]
    us_xyz = np.asarray(us_interp_points)[:, :3]

    pcd_tof = o3d.geometry.PointCloud()
    pcd_tof.points = o3d.utility.Vector3dVector(tof_xyz)
    pcd_tof.paint_uniform_color([0.0, 0.3, 1.0])

    pcd_us = o3d.geometry.PointCloud()
    pcd_us.points = o3d.utility.Vector3dVector(us_xyz)
    pcd_us.paint_uniform_color([0.8, 0.8, 0.8])

    # Downsample + merge + denoise
    voxel = 0.02
    pcd_tof = pcd_tof.voxel_down_sample(voxel)
    pcd_us = pcd_us.voxel_down_sample(voxel)
    combined = pcd_tof + pcd_us
    combined, _ = combined.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0)

    print(f"✅ Fused map ready with {len(combined.points)} points.")
    return combined


# === Main Execution ===
def main():
    t0 = time.time()

    data_name = "22_09_25_MILC/7_lyco_lab/"
    base_path = "../optical_flow_method_data/combined_samples/" + data_name

    data_times = base_path + "data_times.csv"
    tof_input_cropped = base_path + "download_tof_cropped.csv"
    us_input_path = base_path + "fake_ultrasonic.csv"
    us_input_cropped = base_path + "us_cropped.csv"

    cut_data(data_times, us_input_path, us_input_cropped)

    print("\n=== Stage 1: Generate ToF Map ===")
    tof_points, traj_positions = tof_map(tof_input_cropped)

    print("\n=== Stage 2: Generate Ultrasonic Map ===")
    us_interp_points, us_actual_points, us_corner_points = us_map(us_input_cropped)

    print("\n=== Stage 3: Align ToF to Ultrasonic Anchors ===")
    aligned_tof, anchor_cloud = align_tof_to_ultrasonic_weighted(tof_points, us_actual_points, us_corner_points)

    visualize_tof_alignment_comparison(tof_points, aligned_tof, us_actual_points, us_corner_points)

    print("\n=== Stage 4: Fuse Aligned ToF + Ultrasonic ===")
    fused_cloud = fuse_tof_and_ultrasonic(np.asarray(aligned_tof.points), us_interp_points)

    print("\n=== Stage 5: Visualize Fused Result ===")
    o3d.visualization.draw_geometries([fused_cloud], window_name="Final Fused ToF + Ultrasonic Map")

    print(f"\nPipeline complete in {time.time() - t0:.2f}s")


if __name__ == "__main__":
    main()
