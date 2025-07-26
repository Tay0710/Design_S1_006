import math
import numpy as np
import open3d as o3d

# Generate data to simulate the drone moving along x axis and tracking two walls
def generate_parallel_line_data(steps = 20, start_pos = 0.1, spacing = 0.04):
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
        data.append((x, y, z, yaw, distances))
    return data

# Convert position and distances into 3D points
def compute_cloud_points(x, y, z, yaw, distances):
    # Yaw is not important as it is currently fixed
    front, back, left, right, top, bottom = [d / 1000 for d in distances]

    front_pt = (x, y + front, z)
    back_pt = (x, y - back, z)
    # Only two planes
    return [front_pt, back_pt]

if __name__ == "__main__":
    data = generate_parallel_line_data()
    cloud_pts = []

    for x, y, z, yaw, distances in data:
        cloud_pts.extend(compute_cloud_points(x, y, z, yaw, distances))

    # Create the point cloud object to store points in space
    p_c = o3d.geometry.PointCloud()
    # Assign points to the point cloud
    p_c.points = o3d.utility.Vector3dVector(np.array(cloud_pts))
    # Create coordinate axis (X = red, Y = green, Z = blue)
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    # Display the point cloud
    o3d.visualization.draw_geometries([p_c, axis], window_name="Point Cloud - Parallel Walls")

    # Estimate the normals (direction a point is facing) for meshing
    p_c.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))
    # Create a mesh using Poisson surface reconstruction (tries to fill in a smooth surface)
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(p_c, depth=8)
    # Crop (limit) mesh to original point cloud bounds
    bbox = p_c.get_axis_aligned_bounding_box()
    mesh_crop = mesh.crop(bbox)
    # Display the final mesh
    o3d.visualization.draw_geometries([mesh_crop, axis], window_name="Poisson Mesh - Parallel Walls")

    # Save outputs to easily open later
    o3d.io.write_point_cloud("parallel_pointcloud.ply", p_c)
    o3d.io.write_triangle_mesh("parallel_mesh.ply", mesh_crop)
