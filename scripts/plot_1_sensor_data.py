import math
import numpy as np
import open3d as o3d

# Clean the test data line by line
def read_line(line):
    #Remove unnecessary brackets and white spaces
    clean = line.strip()[1:-1]
    # print("Data: ", clean)
    
    #Split using delimiter ','
    values = clean.split(',')
    
    index = values[0]
    x, y, z, = map(float, values[1:4])
    yaw = float(values[4])
    distance = float(values[5])
    
    print("Index:", index)
    print("Position (x, y, z): ", x, y, z)
    print("Yaw:", yaw)
    print("Distance:", distance)
    
    return index, x, y, z, yaw, distance

# Read in the test data file
def read_file(filepath):
    data = []
    with open(filepath, 'r') as file:
        # Skip header line
        next(file)
        for line in file:
            index, x, y, z, yaw, distance = read_line(line)
            data.append((x, y, z, yaw, distance))     
    return data
            
# Convert position and distances into 3D points
def compute_cloud_points(x, y, z_center, yaw_deg, distance_mm, z_steps=6, height=0.6):
    front = distance_mm / 1000  # Convert mm to meters
    yaw_rad = math.radians(yaw_deg)
    dx = front * math.cos(yaw_rad)
    dy = front * math.sin(yaw_rad)

    z_vals = np.linspace(z_center - height / 2, z_center + height / 2, z_steps)
    front_pts = [(x + dx, y + dy, z) for z in z_vals]
    return front_pts

if __name__ == "__main__":
    data = read_file('../sensor_logs/2025-07-27_22-39-57.csv')  # Replace with your actual file
    cloud_pts = []

    for x, y, z, yaw, distance in data:
        cloud_pts.extend(compute_cloud_points(x, y, z, yaw, distance))

    p_c = o3d.geometry.PointCloud()
    p_c.points = o3d.utility.Vector3dVector(np.asarray(cloud_pts, dtype=np.float64))
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    o3d.visualization.draw_geometries([p_c, axis], window_name="Point Cloud")

    p_c.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(p_c, depth=8)
    bbox = p_c.get_axis_aligned_bounding_box()
    mesh_crop = mesh.crop(bbox)
    o3d.visualization.draw_geometries([mesh_crop, axis], window_name="Poisson Mesh")

    o3d.io.write_point_cloud("single_sensor_pointcloud.ply", p_c)
    o3d.io.write_triangle_mesh("single_sensor_mesh.ply", mesh_crop)

