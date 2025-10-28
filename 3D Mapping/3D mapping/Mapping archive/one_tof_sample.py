import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def tof_point(d, theta_x_deg, theta_y_deg, drone_pos=(0,0,0)):
    """
    Compute world coordinates of a ToF point using arctan form for theta_r.
    """
    if d is None:
        return None

    tx = np.tan(np.deg2rad(theta_x_deg))
    ty = np.tan(np.deg2rad(theta_y_deg))

    # 1. Compute resultant angle theta_r
    theta_r = np.arctan(np.sqrt(tx**2 + ty**2))

    if tx == 0 and ty == 0:
        x_local, y_local = 0.0, 0.0
    else:
        # 2. Horizontal displacement
        r_xy = d * np.sin(theta_r)
        denom = np.sqrt(tx**2 + ty**2)
        x_local = r_xy * (tx / denom)
        y_local = r_xy * (ty / denom)

    # 3. Vertical displacement
    z_local = d * np.cos(theta_r)

    # 4. Translate to world frame
    return (
        drone_pos[0] + x_local,
        drone_pos[1] + y_local,
        drone_pos[2] - z_local
    )

def build_points(distances, drone_pos):
    """
    Build ToF points + labels for all 16 zones using manual theta_x, theta_y mapping.
    """
    cell_angles = {
        3:  (-30, -30),
        2:  (-10, -30),
        1:  (10,  -30),
        0:  (30,  -30),

        7:  (-30, -10),
        6:  (-10, -10),
        5:  (10,  -10),
        4:  (30,  -10),

        11: (-30,  10),
        10: (-10,  10),
        9:  (10,   10),
        8:  (30,   10),

        15: (-30,  30),
        14: (-10,  30),
        13: (10,   30),
        12: (30,   30),
    }

    points, labels = [], []
    for idx, (tx, ty) in cell_angles.items():
        d = distances[idx]
        pt = tof_point(d, tx, ty, drone_pos)
        if pt:
            points.append(pt)
            labels.append(f"D{idx}")

    return np.array(points), labels

def visualize_open3d(points, drone_pos):
    geoms = []

    # ToF points (blue)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    pc.paint_uniform_color([0, 0, 1])
    geoms.append(pc)

    # Small cubes at ToF points
    for pt in points:
        cube = o3d.geometry.TriangleMesh.create_box(width=0.05, height=0.05, depth=0.05)
        cube.translate(np.array(pt) - np.array([0.025,0.025,0.025]))
        cube.paint_uniform_color([0, 0, 1])
        geoms.append(cube)

    # Big red sphere for drone
    drone_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    drone_marker.translate(drone_pos)
    drone_marker.paint_uniform_color([1, 0, 0])
    geoms.append(drone_marker)

    # Axis at world origin
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    geoms.append(axis)

    o3d.visualization.draw_geometries(geoms, window_name="ToF + Drone (Open3D)")

def visualize_matplotlib(points, labels, drone_pos):
    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111, projection="3d")

    # Plot ToF points
    ax.scatter(points[:,0], points[:,1], points[:,2], c="blue", marker="s", s=50, label="ToF points")

    # Add labels for ToF points
    for (x, y, z), label in zip(points, labels):
        ax.text(x, y, z, label, color="blue")

    # Plot drone
    ax.scatter(drone_pos[0], drone_pos[1], drone_pos[2], c="red", s=100, marker="o", label="Drone")
    ax.text(drone_pos[0], drone_pos[1], drone_pos[2], f"Drone {drone_pos}", color="red")

    # Labels and formatting
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("ToF + Drone Positions (Matplotlib)")
    ax.legend()
    plt.show()

def main():
    # Load drone position (first line of trajectory CSV)
    traj = pd.read_csv("../optical_flow_method_data/xy_velocities_to_world_frame.csv")
    drone_pos = (
        traj["pos_world_x"].iloc[0],
        traj["pos_world_y"].iloc[0],
        traj["pos_world_z"].iloc[0],
    )

    # Example ToF row
    tof_row = ["19.013483","X","62","44","1108","1171","X","1090","X","X","X","1146","X","X","X","X","1232"]

    # Convert mm → m or None
    distances = [None if d=="X" else float(d)/1000.0 for d in tof_row[1:]]

    # Build points
    points, labels = build_points(distances, drone_pos)

    # Visualize
    visualize_open3d(points, drone_pos)
    visualize_matplotlib(points, labels, drone_pos)

if __name__ == "__main__":
    main()
