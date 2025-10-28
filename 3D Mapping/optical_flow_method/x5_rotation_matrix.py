"""
x5_rotation_matrix.py
---------------------
Stage 5 of the ELEC5550 Indoor 3D Mapping Design Project (2025) position pipeline.

Purpose:
    Estimate time-varying orientation from IMU data and export 3×3 rotation matrices.

Overview:
    Runs an AHRS filter on IMU gyroscope and accelerometer data to compute
    orientation over time. Outputs per-sample rotation matrices (flattened).

    Notes: Includes an optional matplotlib 3D animation for visualization.

Usage:
    Called from x0_position_pipeline.py during Stage 5.

Inputs:
    IMU CSV
        Columns:
            time (s), gyro_x (deg/s), gyro_y (deg/s), gyro_z (deg/s),
            accel_x (g), accel_y (g), accel_z (g)

Outputs:
    ../optical_flow_method_data/rotation_matrices.csv
        Columns:
            time, r00, r01, r02, r10, r11, r12, r20, r21, r22
"""

import numpy as np
import imufusion
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

def load_sensor_data(input_csv):
    """Load IMU CSV with columns: time, gyro_x..z [deg/s], accel_x..z [g]."""
    
    data = np.genfromtxt(input_csv, delimiter=",", skip_header=1)
    
    timestamp = data[:, 0]
    gyroscope = data[:, 1:4]       # [deg/s]
    accelerometer = data[:, 4:7]   # [g]
    
    return timestamp, gyroscope, accelerometer

def run_ahrs(timestamp, gyroscope, accelerometer):
    """Run AHRS and return flattened 3×3 rotation matrices with time."""
    
    sample_rate = int(round(1.0 / np.mean(np.diff(timestamp))))
    print("Sample Rate: ", sample_rate)

    delta_time = np.diff(timestamp, prepend=timestamp[0])

    offset = imufusion.Offset(sample_rate)
    ahrs = imufusion.Ahrs()

    ahrs.settings = imufusion.Settings(
        imufusion.CONVENTION_NWU,
        0.5,               # gain
        250,               # gyroscope range
        2,                 # acceleration rejection
        0,                 # magnetic rejection
        1 * sample_rate,   # recovery trigger period
    )

    rot_mats = []

    for i in range(len(timestamp)):
        gyroscope[i] = offset.update(gyroscope[i])
        
        ahrs.update_no_magnetometer(gyroscope[i], accelerometer[i], delta_time[i])
        R = ahrs.quaternion.to_matrix()
        row = R.flatten()
        row = np.insert(row, 0, timestamp[i])
        rot_mats.append(row)

    return np.array(rot_mats)

def save_rotation_matrices(rot_mats, output_csv):
    """Save rotation matrices to CSV."""
    
    header = ",".join([f"r{i}{j}" for i in range(3) for j in range(3)])
    final_header = "time," + header
    np.savetxt(output_csv, rot_mats, delimiter=",", header=final_header, comments="")
    
    print(f"Saved rotation matrices to {output_csv}")

def animate_rotation(rot_mats):
    """Animate body axes using the rotation matrices."""
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([-1, 1])
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")

    def update(frame):
        ax.cla()
        ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([-1, 1])
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")

        R = rot_mats[frame*50][1:].reshape(3, 3)
        origin = np.zeros(3)
        ax.quiver(*origin, *R[:, 0], color="r", length=1)
        ax.quiver(*origin, *R[:, 1], color="g", length=1)
        ax.quiver(*origin, *R[:, 2], color="b", length=1)
        ax.set_title(f"Frame {frame*50}/{len(rot_mats)}")

    ani = FuncAnimation(fig, update, frames=round(len(rot_mats) / 50), interval = 50)
    plt.show()
    
    return ani

def main(input_csv):
    """Load IMU, run AHRS, save rotation matrices, animate axes."""
    
    output_csv = "../optical_flow_method_data/rotation_matrices.csv"

    timestamp, gyro, accel = load_sensor_data(input_csv)
    rot_mats = run_ahrs(timestamp, gyro, accel)
    save_rotation_matrices(rot_mats, output_csv)

    _ = animate_rotation(rot_mats)

if __name__ == "__main__":
    main()