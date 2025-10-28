import math
import numpy as np
import open3d as o3d
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ======================================================
# === Wall simulation + point cloud (unchanged)
# ======================================================

# Generate data to simulate the drone moving along x axis and tracking two walls
def generate_parallel_line_data(steps=20, start_pos=0.1, spacing=0.04):
    wall_data = []
    for i in range(steps):
        x = start_pos + i * spacing
        y = 0.0
        z = 0.5
        yaw = 90.0
        front = 0.9 * 1000
        back = 0.1 * 1000
        left = right = top = bottom = 0
        distances = [front, back, left, right, top, bottom]
        wall_data.append((x, y, z, yaw, distances))
    return wall_data

# Convert position and distances into 3D points
def compute_cloud_points(x, y, z_center, yaw, distances, z_steps=6, height=0.6):
    front, back, _, _, _, _ = [d / 1000 for d in distances]

    z_min = z_center - height / 2
    z_max = z_center + height / 2
    z_vals = np.linspace(z_min, z_max, z_steps)

    front_pts = [(x, y + front, z) for z in z_vals]
    back_pts = [(x, y - back, z) for z in z_vals]

    return front_pts + back_pts

# ======================================================
# === Trajectory plotting (new function)
# ======================================================

def plot_trajectory(csv_path):
    """Load and plot drone trajectory from world-frame CSV file."""
    data = pd.read_csv(csv_path)

    # --- 2D XY plot ---
    plt.figure(figsize=(6, 6))
    plt.plot(
        data["pos_world_x"], data["pos_world_y"],
        marker="o", markersize=2, linestyle="-", label="Trajectory"
    )
    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.title("Drone XY Position from World Frame Integration")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.show()

    # --- 3D plot ---
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(
        data["pos_world_x"], data["pos_world_y"], data["pos_world_z"],
        label="3D Trajectory"
    )
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Drone 3D Position Trajectory")
    ax.legend()
    plt.show()

# ======================================================
# === Main
# ======================================================

if __name__ == "__main__":
    # --- Point cloud simulation ---
    wall_data = generate_parallel_line_data()
    cloud_pts = []
    for x, y, z, yaw, distances in wall_data:
        cloud_pts.extend(compute_cloud_points(x, y, z, yaw, distances))

    p_c = o3d.geometry.PointCloud()
    p_c.points = o3d.utility.Vector3dVector(np.array(cloud_pts))
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

    # Show raw point cloud
    o3d.visualization.draw_geometries([p_c, axis], window_name="Point Cloud - Parallel Walls")

    # Create mesh from point cloud
    p_c.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(p_c, depth=8)
    bbox = p_c.get_axis_aligned_bounding_box()
    mesh_crop = mesh.crop(bbox)
    o3d.visualization.draw_geometries([mesh_crop, axis], window_name="Poisson Mesh - Parallel Walls")

    # Save outputs
    o3d.io.write_point_cloud("parallel_pointcloud.ply", p_c)
    o3d.io.write_triangle_mesh("parallel_mesh.ply", mesh_crop)

    # --- Plot trajectory from CSV ---
    csv_path = "../optical_flow_method_data/xy_velocities_to_world_frame.csv"
    plot_trajectory(csv_path)
