"""
tof_map_with_rotation.py
-----------------------

Generates a 3D point cloud and mesh of the environment from Time-of-Flight (ToF) sensor data,
mapped onto drone positions derived from integrated velocity estimates and IMU orientation.

Overview:
    - The ToF sensor outputs distances for a 4×4 (up/down) or 8×8 (side) grid of zones at each timestep.
    - Each zone corresponds to a fixed angular offset in the body frame.
    - For each ToF frame, the drone’s world position and orientation (rotation matrix) are obtained
      at the nearest matching timestamp.
    - Distances are projected into body-frame coordinates, then rotated into the world frame.
    - All ToF points are accumulated into a single point cloud representing the mapped scene.
    - Two visualisation backends are provided:
        * Open3D (interactive 3D point cloud + mesh + trajectory)
        * Matplotlib (3D scatter plot with text labels)

Inputs:
    - xy_velocities_to_world_frame.csv
        Columns: 
            time (s), v_world_x, v_world_y, v_world_z,
            pos_world_x, pos_world_y, pos_world_z
    - rotation_matrices.csv
        Columns:
            time (s), r00 … r22 (flattened 3×3 rotation matrix per row)
    - download_tof_cropped.csv
        Columns: 
            time, type, D0 … (distances in mm, invalid entries are "X")

Outputs:
    - Interactive Open3D window with:
        * Blue ToF points
        * Red drone trajectory line
        * Red sphere marking final drone position
    - Matplotlib 3D plot with labeled points and trajectory
"""

import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt

# == Sensor Offsets ==
offsetD = np.array([0.019, 0.0, 0.0]) # [x-offset (m), y-offset (m), z-offset (m)]
offsetU = np.array([0.021, 0.0, 0.132])
offsetL = np.array([0.012, 0.080, 0.035])
offsetR = np.array([0.040, -0.052, 0.035])
# offsetD = np.array([0,0,0]) # [x-offset (m), y-offset (m), z-offset (m)]
# offsetU = np.array([0,0,0])
# offsetL = np.array([0,0,0])
# offsetR = np.array([0,0,0])

# === Load rotation matrices ===
def load_rotation_matrices(rot_csv):
    rot = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=range(1,10))
    times = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=(0,))
    return times, rot.reshape(-1, 3, 3)

# === ToF projection into body frame ===
def tof_point_body(d, theta_x_deg, theta_y_deg):
    if d is None:
        return None

    tx = np.tan(np.deg2rad(theta_x_deg))
    ty = np.tan(np.deg2rad(theta_y_deg))

    theta_r = np.arctan(np.sqrt(tx**2 + ty**2))

    if tx == 0 and ty == 0:
        x_local, y_local = 0.0, 0.0
    else:
        r_xy = d * np.sin(theta_r)
        denom = np.sqrt(tx**2 + ty**2)
        x_local = r_xy * (tx / denom)
        y_local = r_xy * (ty / denom)

    z_local = d * np.cos(theta_r)

    # sensor faces forward: +x, sideways: +y, vertical: +z
    return np.array([x_local, y_local, z_local])  # -z so that "down" is negative

def tof_point_body_2(d, theta_x_deg, theta_y_deg):
    if d is None:
        return None

    pitch = -np.deg2rad(theta_x_deg) 
    roll =  np.deg2rad(theta_y_deg)

    rot_mat = np.array([[np.cos(pitch), 0, -np.sin(pitch)],
                        [np.sin(pitch)*np.sin(roll), np.cos(roll), np.cos(pitch)*np.sin(roll)],
                        [np.sin(pitch)*np.cos(roll), -np.sin(roll), np.cos(pitch)*np.cos(roll)]])
    
    ToF_distance = np.array([0, 0, d])

    coords = rot_mat @ ToF_distance

    return coords

def build_points_down(distances):
    # cell_angles = {
    #     3:  (-30, 30), 2: (-10, 30), 1: (10, 30), 0: (30, 30),
    #     7:  (-30, 10), 6: (-10, 10), 5: (10, 10), 4: (30, 10),
    #     11: (-30, -10), 10:(-10, -10),  9:(10, -10),  8:(30, -10),
    #     15: (-30, -30), 14:(-10, -30), 13:(10, -30),  12:(30, -30),
    # }
    fov = 60.0
    pitch = fov / 3.0  # 20 degrees between each pixel 
    points = []
    R, G, B = 32, 214, 96
    for row in range(4):
        for col in range(4):
            idx = row * 4 + col
            d = distances[idx]
            if d is None or d < 0.2:
                continue
            theta_x = -(col - 1.5) * pitch
            theta_y = -(row - 1.5) * pitch
            local = tof_point_body_2(d, theta_x, theta_y)
            if local is None:
                continue
            lx, ly, lz = local
            pt = np.array([-ly, -lx, -lz, R, G, B])
            pt[0:3] += offsetD
            points.append(pt)
    return points

def build_points_up(distances):
    fov = 60.0
    pitch = fov / 3.0  # 20 degrees between each pixel
    points = []
    R, G, B = 141, 205, 240
    for row in range(4):
        for col in range(4):
            idx = row * 4 + col
            d = distances[idx]
            if d is None or d < 0.2:
                continue
            theta_x = -(col - 1.5) * pitch
            theta_y = -(row - 1.5) * pitch
            local = tof_point_body_2(d, theta_x, theta_y)
            if local is None:
                continue
            lx, ly, lz = local
            pt = np.array([ly, -lx, lz, R, G, B])
            pt[0:3] += offsetU
            points.append(pt)
    return points

def build_points_side(distances, orientation):
    fov = 60.0
    pitch = fov / 7.0  # ~8.5714 degrees between each pixel
    points = []
    for row in range(8):
        for col in range(8):
            idx = row * 8 + col
            d = distances[idx]
            if d is None or d < 0.2:
                continue
            theta_x = -(col - 3.5) * pitch
            theta_y = -(row - 3.5) * pitch
            local = tof_point_body_2(d, theta_x, theta_y)
            if local is None:
                continue
            lx, ly, lz = local
            if orientation == "L":
                R, G, B = 175, 32, 214
                pt = np.array([-lx, lz, ly, R, G, B])  # left sensor looks -Y
                pt[0:3] += offsetL
            else:  # "R"
                R, G, B = 255, 0, 0
                pt = np.array([lx, -lz, ly, R, G, B])   # right sensor looks +Y
                pt[0:3] += offsetR
            points.append(pt)
    return points

# === Rotation into world frame ===
def rotate_point_to_world(local_vec, rot_mat, drone_pos):
    world_vec = rot_mat @ local_vec[:3]
    return (
        drone_pos[0] + world_vec[0],
        drone_pos[1] + world_vec[1],
        drone_pos[2] + world_vec[2],
        local_vec[3], # Include RGB
        local_vec[4],
        local_vec[5]
    )

# === Visualisation ===
def visualize_open3d(points, drone_positions):
    geoms = []
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points[:, :3])
    pc.colors = o3d.utility.Vector3dVector(points[:, 3:6]/255)
    geoms.append(pc)

    traj = o3d.geometry.LineSet()
    traj.points = o3d.utility.Vector3dVector(drone_positions)
    traj.lines = o3d.utility.Vector2iVector([[i, i+1] for i in range(len(drone_positions)-1)])
    traj.colors = o3d.utility.Vector3dVector([[0, 0, 1] for _ in range(len(drone_positions)-1)])
    geoms.append(traj)

    drone_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
    drone_marker.translate(drone_positions[-1])
    drone_marker.paint_uniform_color([0, 0, 1])
    geoms.append(drone_marker)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    geoms.append(axis)

    o3d.visualization.draw_geometries(geoms, window_name="Full ToF Mapping")

def visualize_matplotlib(points, drone_positions):
    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection="3d")

    points = np.array(points)
    drone_positions = np.array(drone_positions)

    ax.scatter(points[:,0], points[:,1], points[:,2], c="blue", marker="s", s=10, label="ToF points")
    ax.plot(drone_positions[:,0], drone_positions[:,1], drone_positions[:,2], c="red", label="Drone trajectory")
    ax.scatter(drone_positions[-1,0], drone_positions[-1,1], drone_positions[-1,2],
               c="red", s=100, marker="o", label="Drone (final)")
    ax.text(drone_positions[-1,0], drone_positions[-1,1], drone_positions[-1,2],
            f"Drone {tuple(drone_positions[-1])}", color="red")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("ToF + Drone Mapping (Matplotlib)")
    plt.axis('equal')
    ax.legend()
    plt.show()

def main(tof_input_cropped):
    # Load trajectory + ToF + rotation data
    traj = pd.read_csv("../optical_flow_method_data/xy_velocities_to_world_frame.csv")
    tof = pd.read_csv(tof_input_cropped)
    times_mat, rot_mats = load_rotation_matrices("../optical_flow_method_data/rotation_matrices.csv")

    all_points = []
    drone_positions = []

    traj_time = traj["time (s)"].values
    tof_time = tof["time"].values

    # Build drone trajectory positions
    for i in range(len(traj)):
        drone_pos = (
            traj["pos_world_x"].iloc[i],
            traj["pos_world_y"].iloc[i],
            traj["pos_world_z"].iloc[i],
        )
        drone_positions.append(drone_pos)

    # For each ToF frame
    for i in range(len(tof)):
        t_type = tof["type"].iloc[i]
        if t_type not in ["D", "U", "L", "R"]:
            continue

        tof_t = tof_time[i]
        match_idx = np.searchsorted(traj_time, tof_t, side="right")
        if match_idx >= len(traj):
            continue

        # Get drone position
        drone_pos = (
            traj["pos_world_x"].iloc[match_idx],
            traj["pos_world_y"].iloc[match_idx],
            traj["pos_world_z"].iloc[match_idx],
        )

        # Get nearest rotation matrix
        rot_idx = np.searchsorted(times_mat, tof_t, side="right")
        if rot_idx >= len(rot_mats):
            rot_mat = rot_mats[-1]
        else:
            rot_mat = rot_mats[rot_idx]

        # Load distances
        if t_type in ["D", "U"]:
            distances = [
                None if str(d) == "X" else float(d) / 1000.0
                for d in tof.iloc[i, 2:18]
            ]
            if t_type == "D":
                local_pts = build_points_down(distances)
            else:
                local_pts = build_points_up(distances)
        else:  # L or R
            distances = [
                None if str(d) == "X" else float(d) / 1000.0
                for d in tof.iloc[i, 2:66]
            ]
            local_pts = build_points_side(distances, orientation=t_type)

        # Rotate + translate into world frame
        world_pts = [rotate_point_to_world(lp, rot_mat, drone_pos) for lp in local_pts]
        all_points.extend(world_pts)
    
    print(offsetD)

    # Visualise
    if len(all_points) == 0:
        print("⚠️ No ToF points were generated, skipping visualisation.")
        return

    visualize_open3d(np.array(all_points), drone_positions)

    # Attempt to filter
    ####        pcd = o3d.geometry.PointCloud()
    
    ####        pcd.points = o3d.utility.Vector3dVector(np.array(all_points)[:, :3])
    # Example with colors (assuming 'points' has 6 columns: x, y, z, R, G, B)
    # all_points[:, 3:6] / 255.0 # Normalize if needed
    ####        pcd.colors = o3d.utility.Vector3dVector(np.array(all_points)[:, 3:6]/255)
    ####        o3d.visualization.draw_geometries([pcd], window_name="Original Point Cloud")

    # Apply statistical outlier removal
    # 20 neighbors and a standard deviation ratio of 2.0 are common starting points
    #####      cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)

    # `cl` contains the inlier points (cleaned point cloud)
    # `ind` contains the indices of the inlier points
    # Visualize the original and filtered point clouds
    ####       o3d.visualization.draw_geometries([cl], window_name="Filtered Point Cloud (Inliers)")

    # To visualize the removed outliers, you can extract them using the `ind` variable
    ####       outlier_pcd = pcd.select_by_index(ind, invert=True)
    ####       o3d.visualization.draw_geometries([outlier_pcd], window_name="Removed Outliers")

    ####       visualize_matplotlib(all_points, drone_positions)

if __name__ == "__main__":
    main()
