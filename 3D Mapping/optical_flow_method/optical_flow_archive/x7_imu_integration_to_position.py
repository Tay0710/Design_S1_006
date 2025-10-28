"""
x7_imu_integration_to_position.py
------------------------
Estimates velocity and position directly from IMU data 
(gyroscope + accelerometer) with motion detection.

Overview:
    - Runs imufusion AHRS on IMU data.
    - Computes linear acceleration (gravity removed).
    - Detects "moving" vs "still" periods based on acceleration norm.
    - Integrates acceleration → velocity (clamped to zero when still).
    - Integrates velocity → position.
    - Detrends velocity signals to mitigate drift.
    - Saves results to CSV and plots 2D trajectory.

Notes:
    - Gravity removal:
        By default we use ahrs.earth_acceleration (already gravity-compensated).
        Optionally, subtract explicit gravity vector if tuning requires.
    - Motion detection:
        Threshold-based (acceleration norm > 0.2 m/s²).
        Smoothed with a margin to avoid chattering.
    - Drift:
        Linear detrend applied to each velocity component.

Inputs:
    - IMU_combined_square2.csv
        Columns:
            time (s), gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z
        Units:
            Gyroe in deg/s
            Accel in g

Outputs:
    - imu_position.csv
        Columns:
            time, vx, vy, vz, px, py, pz
        Units:
            velocity in m/s, position in m
"""

from dataclasses import dataclass
from scipy.signal import detrend
import imufusion
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import csv

@dataclass
class IsMovingPeriod:
    start_index: int = -1
    stop_index: int = -1

# === Function: Process sensor data ===
def process_data(filepath):
    """Load CSV, run AHRS, return timestamp, accel (m/s²), motion detection"""
    # === Import sensor data ===
    data = np.genfromtxt(filepath, delimiter=",", skip_header=1)

    timestamp = data[:, 0]
    gyroscope = data[:, 1:4]
    accelerometer = data[:, 4:7]

    # === Calculate sample rate ===
    sample_rate = 1.0 / np.mean(np.diff(timestamp))
    print("Sample Rate: ", sample_rate)

    # === Instantiate AHRS algorithms ===
    offset = imufusion.Offset(int(sample_rate))
    ahrs = imufusion.Ahrs()

    # === Tuning Variables ===
    gain = 1
    gyro_range = 2000 # Set range of the gyro
    accel_rej = 2 # Set max of the accel
    mag_rej = 0
    rej_timeout = 4 * int(sample_rate)
    motion_threshold = 0.2
    smoothing_margin = int(0.3 * sample_rate)
                        
    ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,
                                    gain,
                                    gyro_range,
                                    accel_rej,
                                    mag_rej,
                                    rej_timeout)

    # === Gravity removal constants ===
    G = 9.80665
    GRAVITY_G_NWU = np.array([0.0, 0.0, -1.0])  # NWU: Up=+Z

    # === Process sensor data ===
    delta_time = np.diff(timestamp, prepend=timestamp[0])

    euler = np.empty((len(timestamp), 3))
    internal_states = np.empty((len(timestamp), 3))
    acceleration = np.empty((len(timestamp), 3))

    for index in range(len(timestamp)):
        gyroscope[index] = offset.update(gyroscope[index])

        # Convert accel to g for imufusion (accel is already in g)
        accel_g = accelerometer[index]
        ahrs.update_no_magnetometer(gyroscope[index], accel_g, delta_time[index])

        euler[index] = ahrs.quaternion.to_euler()
        internal = ahrs.internal_states
        internal_states[index] = np.array([
            internal.acceleration_error,
            internal.accelerometer_ignored,
            internal.acceleration_recovery_trigger
        ])

        # Gravity removal: earth_acceleration is in g → subtract gravity vector → m/s²
        earth_accel_g = ahrs.earth_acceleration
        linear_accel_mps2 = (earth_accel_g) * G
        # linear_accel_mps2 = (earth_accel_g - GRAVITY_G_NWU) * G # IDK IF I SHOULD BE REMOVING THIS SUBTRACTION
        acceleration[index] = linear_accel_mps2

        if index % 500 == 0:
            print(f"[{index}] lin_acc (m/s^2) = {acceleration[index]}, norm = {np.linalg.norm(acceleration[index]):.2f}")

    # === Acceleration statistics ===
    acc_norms = np.linalg.norm(acceleration, axis=1)
    print(f"\n📊 Acceleration stats:")
    print(f"  Mean norm: {np.mean(acc_norms):.2f} m/s²")
    print(f"  Min norm:  {np.min(acc_norms):.2f} m/s²")
    print(f"  Max norm:  {np.max(acc_norms):.2f} m/s²")
    print(f"  Std dev:   {np.std(acc_norms):.2f} m/s²")

    # === Identify moving periods ===
    is_moving = np.zeros(len(timestamp), dtype=bool)
    for index in range(len(timestamp)):
        acc_norm = np.linalg.norm(acceleration[index])
        is_moving[index] = acc_norm > motion_threshold

    # Smooth motion detection mask
    for index in range(len(timestamp) - smoothing_margin):
        is_moving[index] = np.any(is_moving[index:(index + smoothing_margin)])
    for index in range(len(timestamp) - 1, smoothing_margin, -1):
        is_moving[index] = np.any(is_moving[(index - smoothing_margin):index])

    print(f"\n✅ Motion detection summary:")
    print(f"Total samples: {len(is_moving)}")
    print(f"Moving samples: {np.count_nonzero(is_moving)}")
    print(f"Still samples: {len(is_moving) - np.count_nonzero(is_moving)}\n")

    return timestamp, delta_time, acceleration, is_moving

# === Function: Acceleration → Velocity ===
def accel_to_velocity(timestamp, acceleration, delta_time, is_moving):
    # === Calculate velocity ===
    velocity = np.zeros((len(timestamp), 3))
    for index in range(len(timestamp)):
        if is_moving[index]:
            velocity[index] = velocity[index - 1] + delta_time[index] * acceleration[index]
        else:
            velocity[index] = np.zeros(3)  # clamp to zero during still

    # === Find moving periods ===
    is_moving_diff = np.diff(is_moving, append=is_moving[-1])

    is_moving_periods = []
    period = IsMovingPeriod()
    for index in range(len(timestamp)):
        if period.start_index == -1 and is_moving_diff[index] == 1:
            period.start_index = index
        elif period.start_index != -1 and is_moving_diff[index] == -1:
            period.stop_index = index
            is_moving_periods.append(period)
            period = IsMovingPeriod()

    # === Remove integral drift from velocity ===
    for i in range(3):
        velocity[:, i] = detrend(velocity[:, i], type='linear')
    return velocity

# === Function: Velocity → Position ===
def velocity_to_position(timestamp, velocity, delta_time):
# === Calculate position ===
    position = np.zeros((len(timestamp), 3))
    for index in range(len(timestamp)):
        position[index] = position[index - 1] + delta_time[index] * velocity[index]
    return position

# === Function: Save CSV ===
def save_csv(filename, timestamp, velocity, position):
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time", "vx", "vy", "vz", "px", "py", "pz"
        ])
        for i in range(len(timestamp)):
            writer.writerow([
                timestamp[i],
                *velocity[i],
                *position[i]
            ])
    print(f"Saved results to {filename}")

# === Main ===
def main(input_csv):
    output_csv = "../optical_flow_method_data/imu_position.csv"
    timestamp, delta_time, acceleration, is_moving = process_data(input_csv)
    velocity = accel_to_velocity(timestamp, acceleration, delta_time, is_moving)
    position = velocity_to_position(timestamp, velocity, delta_time)
    save_csv(output_csv, timestamp, velocity, position)

    # Example XY plot
    plt.figure(figsize=(8, 8))
    plt.plot(position[:, 0], position[:, 1], marker="o", markersize=1, linewidth=1)
        # Start (green) and End (red)
    plt.scatter(position[0, 0], position[0, 1], color="green", s=80, label="Start")
    plt.scatter(position[-1, 0], position[-1, 1], color="red", s=80, label="End")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("2D Trajectory")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()