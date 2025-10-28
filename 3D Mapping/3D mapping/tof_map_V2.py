"""
tof_map_V2.py
-----------------------
Primary map of the ELEC5550 Indoor 3D Mapping Design Project (2025) mapping pipeline.

Purpose:
    Generate a 3D point cloud of the environment from Time of Flight data using the
    drone trajectory and IMU orientation.

Overview:
    For each ToF frame, convert zone distances to body frame coordinates, transform
    them to the world frame using the nearest rotation matrix and position, and
    accumulate all points into a single cloud.
    Provides Open3D and Matplotlib visualisations.

Usage:
    Called from x0_mapping_pipeline_V3.py with a cropped ToF CSV path.

Inputs:
    ../optical_flow_method_data/xy_velocities_to_world_frame.csv
        Columns:
            time (s), v_world_x, v_world_y, v_world_z, pos_world_x, pos_world_y, pos_world_z
    ../optical_flow_method_data/rotation_matrices.csv
        Columns:
            time (s), r00, r01, r02, r10, r11, r12, r20, r21, r22
    ../download_tof_cropped.csv
        Columns:
            time, type, D0 … D15/D63 distances in mm with invalid entries marked X

Outputs:
    Open3D ToF Map
    Matplotlib 3D plot with labeled points and trajectory
    Returns:
        all_points          ToF point cloud data
        drone_positions     Drone trajectory
"""


import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt

# Sensor offsets
offsetD = np.array([0,0,0])
offsetU = np.array([0,0,0])
offsetL = np.array([0,0,0])
offsetR = np.array([0,0,0])

def load_rotation_matrices(rot_csv):
    """Load rotation matrices CSV and return times and matrices shaped (N, 3, 3)."""
    
    rot = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=range(1,10))
    times = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=(0,))
    
    return times, rot.reshape(-1, 3, 3)

def tof_point_body(d, theta_x_deg, theta_y_deg):
    """Project a single ToF range and angular offsets into body-frame XYZ."""
    
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

    return np.array([x_local, y_local, z_local])

def tof_point_body_2(d, theta_x_deg, theta_y_deg):
    """Alternate body-frame projection using a small-angle pitch and roll rotation."""
    
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
    
    """Build 4×4 DOWN-facing ToF points in body frame with color and sensor offset."""
    
    fov = 60.0
    pitch = fov / 3.0
    points = []
    R, G, B = 0, 113, 145
    
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
    """Build 4×4 UP-facing ToF points in body frame with color and sensor offset."""
    
    fov = 60.0
    pitch = fov / 3.0
    points = []
    R, G, B = 98, 200, 211
    
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
    """Build 8×8 LEFT or RIGHT ToF points in body frame with color and sensor offset."""
    
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
                R, G, B = 211, 31, 17
                pt = np.array([lz, ly, lx, R, G, B])
                pt[0:3] += offsetL
            else:
                R, G, B = 244, 122, 0
                pt = np.array([-lz, -ly, lx, R, G, B])
                pt[0:3] += offsetR
            points.append(pt)
            
    return points

def rotate_point_to_world(local_vec, rot_mat, drone_pos):
    """Rotate a body-frame colored point to world frame and translate by drone position."""
    
    world_vec = rot_mat @ local_vec[:3]
    
    return (
        drone_pos[0] + world_vec[0],
        drone_pos[1] + world_vec[1],
        drone_pos[2] + world_vec[2],
        local_vec[3],
        local_vec[4],
        local_vec[5]
    )

def visualise_open3d(points, drone_positions):
    """Open3D viewer for ToF point cloud with trajectory line, final-pose marker, and axes."""
    
    geoms = []
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points[:, :3])
    pc.colors = o3d.utility.Vector3dVector(points[:, 3:6] / 255.0)
    geoms.append(pc)

    traj = o3d.geometry.LineSet()
    traj.points = o3d.utility.Vector3dVector(drone_positions)
    traj.lines = o3d.utility.Vector2iVector([[i, i+1] for i in range(len(drone_positions)-1)])
    traj.colors = o3d.utility.Vector3dVector([[1.0, 0.176, 0.667] for _ in range(len(drone_positions)-1)])
    geoms.append(traj)

    drone_marker = o3d.geometry.TriangleMesh.create_sphere(radius = 0.1)
    drone_marker.translate(drone_positions[-1])
    drone_marker.paint_uniform_color([1.0, 0.176, 0.667])
    geoms.append(drone_marker)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.2)
    geoms.append(axis)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="ToF Map")
    for g in geoms:
        vis.add_geometry(g)

    opt = vis.get_render_option()
    opt.line_width = 6.0

    vis.run()
    vis.destroy_window()

def visualise_matplotlib(points, drone_positions):
    """Matplotlib 3D scatter of ToF points with trajectory and final-pose marker."""
      
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
    """Load data, build ToF points per frame, transform to world, and visualise."""
    
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
        else:
            distances = [
                None if str(d) == "X" else float(d) / 1000.0
                for d in tof.iloc[i, 2:66]
            ]
            local_pts = build_points_side(distances, orientation=t_type)

        # Rotate and translate into world frame
        world_pts = [rotate_point_to_world(lp, rot_mat, drone_pos) for lp in local_pts]
        all_points.extend(world_pts)

    # Visualise
    if len(all_points) == 0:
        print("⚠️ No ToF points were generated, skipping visualisation.")
        return

    visualise_open3d(np.array(all_points), drone_positions)

    return np.array(all_points), np.array(drone_positions)

if __name__ == "__main__":
    main()
