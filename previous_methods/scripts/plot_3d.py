from format_data import read_file
import open3d as o3d
import numpy as np
import math

def compute_cloud_points(x, y, z, yaw, distances):
    yaw_rad = math.radians(yaw)
    yaw_cos = math.cos(yaw_rad)
    yaw_sin = math.sin(yaw_rad)
    
    front, back, left, right, top, bottom = distances
    
    cloud_pts = []
    
    # Front points
    cloud_pts.append((x + front * yaw_sin, y + front * yaw_cos, z))
    # Back points
    cloud_pts.append((x - back * yaw_sin, y - back * yaw_cos, z))
    # Left points
    cloud_pts.append((x - left * yaw_cos, y + left * yaw_sin, z))
    # Right points
    cloud_pts.append((x + right * yaw_cos, y - right * yaw_sin, z))
    # Top points
    cloud_pts.append((x, y, z + top))
    # Bottom points
    cloud_pts.append((x, y, z - bottom))
        
    return cloud_pts

if __name__ == '__main__':

    # For easy visualisation
    DIRECTION_COLORS = [
        [1, 0, 0],    # Front - red
        [1, 0, 1],    # Back - magenta
        [0, 1, 0],    # Left - green
        [0, 0, 1],    # Right - blue
        [1, 1, 0],    # Top - yellow
        [0, 1, 1]     # Bottom - cyan
    ]
    
    DATA_FILE = '../sensor_logs/left_right_lines.txt'
    timestamps, positions, yaws, distance_sets = read_file(DATA_FILE)

    all_points = []
    all_colors = []

    # Store drone and right-line points
    drone_line_pts = []
    right_line_pts = []

    for pos, yaw, distances in zip(positions, yaws, distance_sets):
        x, y, z = pos
        obstacle_pts = compute_cloud_points(x, y, z, yaw, distances)
        all_points.extend(obstacle_pts)
        all_colors.extend(DIRECTION_COLORS)  # one color per point in order

        # Store center drone path (white)
        drone_line_pts.append([x, y, z])

        # Calculate right offset point (e.g. 0.2m to the right)
        yaw_rad = math.radians(yaw)
        dx = 0.2 * math.sin(yaw_rad)   # perpendicular to yaw
        dy = -0.2 * math.cos(yaw_rad)
        right_line_pts.append([x + dx, y + dy, z])

    # Convert all obstacle points to point cloud
    pcd_obstacles = o3d.geometry.PointCloud()
    pcd_obstacles.points = o3d.utility.Vector3dVector(np.array(all_points, dtype=np.float64))
    pcd_obstacles.colors = o3d.utility.Vector3dVector(np.array(all_colors, dtype=np.float64))

    # Convert drone line to point cloud (white)
    drone_line = o3d.geometry.PointCloud()
    drone_line.points = o3d.utility.Vector3dVector(np.array(drone_line_pts, dtype=np.float64))
    drone_line.paint_uniform_color([1, 1, 1])  # White

    # Convert right parallel line to point cloud (orange)
    right_line = o3d.geometry.PointCloud()
    right_line.points = o3d.utility.Vector3dVector(np.array(right_line_pts, dtype=np.float64))
    right_line.paint_uniform_color([1, 0.5, 0])  # Orange

    # Visualise all
    o3d.visualization.draw_geometries(
        [pcd_obstacles, drone_line, right_line],
        window_name="3D Mapping Visualization",
        width=800,
        height=600
    )
