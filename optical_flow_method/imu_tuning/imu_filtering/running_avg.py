"""
x7_imu_integration_to_position_filtered.py
------------------------------------------
Estimates velocity and position directly from IMU data 
with AHRS orientation, gravity removal, motion detection,
and a running average filter on acceleration.

Overview:
    - Runs imufusion AHRS on IMU data.
    - Computes linear acceleration (gravity removed).
    - Applies running average smoothing to acceleration.
    - Detects "moving" vs "still" periods.
    - Integrates acceleration → velocity (clamped when still).
    - Integrates velocity → position.
    - Detrends velocity signals to mitigate drift.
    - Saves results to CSV and plots 2D trajectory.
"""

from dataclasses import dataclass
from scipy.signal import detrend
import imufusion
import numpy as np
import matplotlib.pyplot as plt
import csv

@dataclass
class IsMovingPeriod:
    start_index: int = -1
    stop_index: int = -1

# === Running average filter ===
def running_average(data, window_size):
    return np.convolve(data, np.ones(window_size)/window_size, mode="valid")

# === Function: Process sensor data ===
def process_data(filepath, window_size=10):
    """Load CSV, run AHRS, return timestamp, filtered accel (m/s²), motion detection"""
    # === Import sensor data ===
    data = np.genfromtxt(filepath, delimiter=",", skip_header=1)
    timestamp = data[:, 0]
    gyroscope = data[:, 1:4]
    accelerometer = data[:, 4:7]

    # === Calculate sample rate ===
    sample_rate = 1600
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

    delta_time = np.diff(timestamp, prepend=timestamp[0])
    acceleration = np.empty((len(timestamp), 3))

    for index in range(len(timestamp)):
        gyroscope[index] = offset.update(gyroscope[index])
        accel_g = accelerometer[index]  # already in g
        ahrs.update_no_magnetometer(gyroscope[index], accel_g, delta_time[index])

        # Convert to m/s² (earth_acceleration already gravity-compensated)
        earth_accel_g = ahrs.earth_acceleration
        acceleration[index] = earth_accel_g * G

    # === Apply running average filter to each axis ===
    x_f = running_average(acceleration[:, 0], window_size)
    y_f = running_average(acceleration[:, 1], window_size)
    z_f = running_average(acceleration[:, 2], window_size)
    acceleration_f = np.column_stack((x_f, y_f, z_f))
    timestamp_f = timestamp[window_size-1:]
    delta_time_f = np.diff(timestamp_f, prepend=timestamp_f[0])

    # === Motion detection ===
    motion_threshold = 0.2
    smoothing_margin = int(0.3 * sample_rate)

    acc_norms = np.linalg.norm(acceleration_f, axis=1)
    is_moving = acc_norms > motion_threshold

    # Smooth mask
    for i in range(len(is_moving) - smoothing_margin):
        is_moving[i] = np.any(is_moving[i:(i+smoothing_margin)])
    for i in range(len(is_moving) - 1, smoothing_margin, -1):
        is_moving[i] = np.any(is_moving[(i-smoothing_margin):i])

    return timestamp_f, delta_time_f, acceleration_f, is_moving

# === Function: Acceleration → Velocity ===
def accel_to_velocity(timestamp, acceleration, delta_time, is_moving):
    velocity = np.zeros((len(timestamp), 3))
    for i in range(1, len(timestamp)):
        if is_moving[i]:
            velocity[i] = velocity[i-1] + delta_time[i] * acceleration[i]
        else:
            velocity[i] = np.zeros(3)

    # Detrend to remove drift
    for j in range(3):
        velocity[:, j] = detrend(velocity[:, j], type='linear')
    return velocity

# === Function: Velocity → Position ===
def velocity_to_position(timestamp, velocity, delta_time):
    position = np.zeros((len(timestamp), 3))
    for i in range(1, len(timestamp)):
        position[i] = position[i-1] + delta_time[i] * velocity[i]
    return position

# === Save CSV ===
def save_csv(filename, timestamp, velocity, position):
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "vx", "vy", "vz", "px", "py", "pz"])
        for i in range(len(timestamp)):
            writer.writerow([timestamp[i], *velocity[i], *position[i]])
    print(f"Saved results to {filename}")

# === Main ===
def main(input_csv):
    output_csv = "../optical_flow_method_data/imu_position_filtered.csv"
    timestamp, delta_time, acceleration, is_moving = process_data(input_csv, window_size=10)
    velocity = accel_to_velocity(timestamp, acceleration, delta_time, is_moving)
    position = velocity_to_position(timestamp, velocity, delta_time)
    save_csv(output_csv, timestamp, velocity, position)

    # === Plot results ===
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    axs[0].plot(timestamp, acceleration[:, 0], label="Ax")
    axs[0].plot(timestamp, acceleration[:, 1], label="Ay")
    axs[0].plot(timestamp, acceleration[:, 2], label="Az")
    axs[0].set_title("Filtered Acceleration")
    axs[0].legend()

    axs[1].plot(timestamp, velocity[:, 0], label="Vx")
    axs[1].plot(timestamp, velocity[:, 1], label="Vy")
    axs[1].plot(timestamp, velocity[:, 2], label="Vz")
    axs[1].set_title("Velocity (Filtered)")
    axs[1].legend()

    axs[2].plot(timestamp, position[:, 0], label="Px")
    axs[2].plot(timestamp, position[:, 1], label="Py")
    axs[2].plot(timestamp, position[:, 2], label="Pz")
    axs[2].set_title("Position (Filtered)")
    axs[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main("../../../optical_flow_method_data/combined_samples/13_09_25_MILC/straight1/download_imu_cropped.csv")
