import math
import numpy as np
import open3d as o3d

# Sample data
sample_data = [
    "(13:10:34:102, 0.1, 0.0, 0.5, 90.0, 200, 300, 100, 100, 150, 50)",
    "(13:10:34:202, 0.2, 0.0, 0.5, 90.0, 190, 310, 100, 100, 150, 50)",
    "(13:10:34:302, 0.3, 0.0, 0.5, 90.0, 180, 320, 100, 100, 150, 50)",
    "(13:10:34:402, 0.4, 0.0, 0.5, 90.0, 170, 330, 100, 100, 150, 50)",
    "(13:10:34:502, 0.5, 0.0, 0.5, 90.0, 160, 340, 100, 100, 150, 50)",
    "(13:10:34:602, 0.6, 0.0, 0.5, 90.0, 150, 350, 100, 100, 150, 50)",
]

# Parse one line of data
def read_line(line):
    clean = line.strip()[1:-1]
    values = clean.split(',')
    timestamp = values[0]
    x, y, z = map(float, values[1:4])
    yaw = float(values[4])
    distances = list(map(int, values[5:11]))
    return timestamp, (x, y, z), yaw, distances

# Compute 3D points from distance sensors
def compute_cloud_points(x, y, z, yaw_deg, distances):
    yaw = math.radians(yaw_deg)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    front, back, left, right, top, bottom = [d / 1000 for d in distances]

    points = [
        (x + front * cos_yaw, y + front * sin_yaw, z),
        (x - back * cos_yaw, y - back * sin_yaw, z),
        (x - left * sin_yaw, y + left * cos_yaw, z),
        (x + right * sin_yaw, y - right * cos_yaw, z),
        (x, y, z + top),
        (x, y, z - bottom),
    ]
    return points

if __name__ == "__main__":
    # Process all data
    all_points = []
    for line in sample_data:
        _, pos, yaw, dists = read_line(line)
        all_points.extend(compute_cloud_points(*pos, yaw, dists))

    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(all_points))

    # Estimate normals (needed for mesh)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))

    # Create mesh using Poisson surface reconstruction
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)

    # Crop mesh to bounding box of the original cloud
    bbox = pcd.get_axis_aligned_bounding_box()
    mesh_crop = mesh.crop(bbox)

    # Add XYZ axis for reference
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

    # Visualize point cloud, mesh, and axes
    o3d.visualization.draw_geometries([pcd, mesh_crop, axis])

    # Save outputs
    o3d.io.write_point_cloud("output_pointcloud.ply", pcd)
    o3d.io.write_triangle_mesh("output_mesh.ply", mesh_crop)
