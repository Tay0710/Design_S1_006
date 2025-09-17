"""
x6_convert_to_world_frame.py
-------------------------------
Transforms body-frame velocities into the world frame and integrates them 
to estimate drone trajectory.

Overview:
    - Optical flow + ToF pipeline produces body-frame planar velocities (v_x, v_y).
    - IMU AHRS produces rotation matrices describing orientation over time.
    - This script rotates velocities into the world frame and integrates them 
      to estimate drone position.

Notes:
    - v_body is extended to 3D with z=0 since optical flow does not capture vertical motion.
    
Pipeline:
    1. Load rotation matrices (IMU orientation).
    2. Load body-frame velocities (optical flow + ToF).
    3. Rotate body velocities into world frame: v_world = R @ v_body.
    4. Integrate v_world using trapezoidal rule to get position.
    5. Save results to CSV and generate diagnostic plots.

Inputs:
    - rotation_matrices.csv
        Columns: r00 … r22 (flattened 3×3 orientation matrix per row).
    - xy_velocities.csv
        Columns: time (s), v_x (m/s), v_y (m/s)

Outputs:
    - xy_velocities_to_world_frame.csv
        Columns:
            time (s),
            v_world_x, v_world_y, v_world_z,
            pos_world_x, pos_world_y, pos_world_z
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def load_rotation_matrices(rot_csv):
    """Load rotation matrices CSV and reshape into (N, 3, 3)."""
    rot = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=(1,2,3,4,5,6,7,8,9))
    times = np.loadtxt(rot_csv, delimiter=",", skiprows=1, usecols=(0))
    return times, rot.reshape(-1, 3, 3)

def load_body_velocities(vel_csv):
    """Load body-frame velocities and timestamps from CSV."""
    vel_df = pd.read_csv(vel_csv)
    times = vel_df["time (s)"].values
    v_body = vel_df[["v_x (m/s)", "v_y (m/s)"]].values
    v_body3 = np.hstack([v_body, np.zeros((len(v_body), 1))])  # add z=0
    return times, v_body3

def match_rotation_matrices_times(rot_mats, times_v, times_mat):
    """Pick closest rotation matrix and store into new np.arr."""
    index = 0
    rot_mats_matched = []
    for t in times_v:
        # Look through rot_mats times to find t
        while index < len(times_mat):
            if times_mat[index] > t:
                rot_mats_matched.append(rot_mats[index])
                break
            index = index + 1
    return rot_mats_matched

def rotate_to_world(rot_mats, v_body3):
    """Rotate body-frame velocities into world frame."""
    v_world = np.zeros_like(v_body3)
    for i in range(len(v_body3)):
        v_world[i] = rot_mats[i] @ v_body3[i]
        v_world[i, 2] = 0.0   # force z-velocity to 0 for every sample
    return v_world

def integrate_velocity(times, v_world):
    """Integrate velocity to get position (trapezoidal)."""
    pos_world = np.zeros_like(v_world)
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        pos_world[i] = pos_world[i - 1] + 0.5 * dt * (v_world[i] + v_world[i - 1])
    return pos_world

def save_results(times, v_world, pos_world, output_csv):
    """Save world-frame velocities and positions to CSV."""
    out = pd.DataFrame({
        "time (s)": times,
        "v_world_x": v_world[:, 0],
        "v_world_y": v_world[:, 1],
        "v_world_z": v_world[:, 2],
        "pos_world_x": pos_world[:, 0],
        "pos_world_y": pos_world[:, 1],
        "pos_world_z": pos_world[:, 2],
    })
    out.to_csv(output_csv, index=False)
    print(f"Saved world-frame velocities and positions to {output_csv}")

def plot_positions_and_velocities(times, v_world, pos_world):
    """Plot positions and velocities in world frame."""
    # === Plot positions ===
    plt.figure(figsize=(12, 5))

    # XY trajectory
    plt.subplot(1, 2, 1)
    plt.plot(pos_world[:, 0], pos_world[:, 1], "-o")
    plt.xlabel("X (m, world)")
    plt.ylabel("Y (m, world)")
    plt.title("Trajectory in World Frame (XY)")
    plt.axis("equal")
    plt.grid(True)

    # XYZ position vs time
    plt.subplot(1, 2, 2)
    plt.plot(times, pos_world[:, 0], label="X (m)")
    plt.plot(times, pos_world[:, 1], label="Y (m)")
    plt.plot(times, pos_world[:, 2], label="Z (m)")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title("World Position vs Time")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    # === Plot velocities ===
    plt.figure(figsize=(8, 5))
    plt.plot(times, v_world[:, 0], label="Vx (m/s)")
    plt.plot(times, v_world[:, 1], label="Vy (m/s)")
    plt.plot(times, v_world[:, 2], label="Vz (m/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("World Velocities vs Time")
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    rot_csv = "../optical_flow_method_data/rotation_matrices.csv"
    vel_csv = "../optical_flow_method_data/xy_velocities.csv"
    output_csv = "../optical_flow_method_data/xy_velocities_to_world_frame.csv"

    # Load data
    times_mat, rot_mats = load_rotation_matrices(rot_csv)
    times_v, v_body3 = load_body_velocities(vel_csv)

    # Match rotation matrix to times from velocity
    rot_mats_matched = match_rotation_matrices_times(rot_mats, times_v, times_mat)

    # Rotate + integrate
    v_world = rotate_to_world(rot_mats_matched, v_body3)
    pos_world = integrate_velocity(times_v, v_world)

    # Save and plot
    save_results(times_v, v_world, pos_world, output_csv)
    plot_positions_and_velocities(times_v, v_world, pos_world)

if __name__ == "__main__":
    main()