from format_data import read_file
import open3d as o3d
import numpy as np
import math

def compute_side_points(position, yaw, distances):
    x, y, z = position
    left, right = distances[2], distances[3]

    yaw_rad = math.radians(yaw)
    yaw_cos = math.cos(yaw_rad)
    yaw_sin = math.sin(yaw_rad)

    # Left: 90° rotation from forward direction
    left_x = x - left * yaw_sin
    left_y = y + left * yaw_cos

    # Right: -90° rotation
    right_x = x + right * yaw_sin
    right_y = y - right * yaw_cos

    return (left_x, left_y, z), (right_x, right_y, z)

def main():
    timestamps, positions, yaws, distance_sets = read_file("../sensor_logs/l_r_5_line_sample.txt")

    left_points = []
    right_points = []

    for pos, yaw, distances in zip(positions, yaws, distance_sets):
        left_pt, right_pt = compute_side_points(pos, yaw, distances)
        left_points.append(left_pt)
        right_points.append(right_pt)

    left_cloud = o3d.geometry.PointCloud()
    right_cloud = o3d.geometry.PointCloud()

    left_cloud.points = o3d.utility.Vector3dVector(np.array(left_points))
    right_cloud.points = o3d.utility.Vector3dVector(np.array(right_points))

    left_cloud.paint_uniform_color([0.5, 0.0, 0.0])   # dark red
    right_cloud.paint_uniform_color([0.0, 0.5, 0.0])  # dark green


    o3d.visualization.draw_geometries([left_cloud, right_cloud])

if __name__ == "__main__":
    main()
