import pandas as pd
import numpy as np
import imufusion as fus

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib import cm
from scipy.interpolate import interp1d

def load_sensor_data(filepath):
    return pd.read_csv(filepath)

def calculate_sample_rate(time):
    time_step = np.mean(np.diff(time))
    sample_rate = 1.0 / time_step
    return sample_rate, time_step

def calculate_orientation(full_data, time_step):
    deg_to_rad = np.pi / 180.0
    gyro = full_data[['Gyroscope X (deg/s)', 'Gyroscope Y (deg/s)', 'Gyroscope Z (deg/s)']].values * deg_to_rad
    accel = full_data[['Accelerometer X (g)', 'Accelerometer Y (g)', 'Accelerometer Z (g)']].values

    ahrs = fus.Ahrs()
    eulers = []
    quaternions = []

    zero_mag = np.array([0.0, 0.0, 0.0])
    for g, a in zip(gyro, accel):
        ahrs.update(g, a, zero_mag, time_step)
        q = fus.Quaternion(np.array([ahrs.quaternion.w, ahrs.quaternion.x, ahrs.quaternion.y, ahrs.quaternion.z]))
        quaternions.append(q)
        eulers.append(q.to_euler())

    return np.array(eulers), quaternions

def rotate_vector_by_quaternion(v, q):
    w, qx, qy, qz = q.w, q.x, q.y, q.z
    q_vec = np.array([qx, qy, qz])
    cross_1 = np.cross(q_vec, v)
    cross_2 = np.cross(q_vec, cross_1)
    return v + 2 * (w * cross_1 + cross_2)

def remove_gravity(acceleration_data, quaternions):
    gravity_global = np.array([0, 0, 9.80665])  # gravity in m/s²
    linear_accels = []
    for a, q in zip(acceleration_data, quaternions):
        gravity = rotate_vector_by_quaternion(gravity_global, q)
        lin_accel = a * 9.80665 - gravity  # convert 'g' to m/s²
        linear_accels.append(lin_accel[:2])  # X and Y only
    return np.array(linear_accels)

def detect_movement(accels_xy, threshold=3.0):
    return np.linalg.norm(accels_xy, axis=1) > threshold  # already in m/s²

def apply_margin(is_moving, sample_rate, margin_time=0.1):
    margin = int(margin_time * sample_rate)
    extended = is_moving.copy()
    for i in range(len(is_moving) - margin):
        if np.any(is_moving[i:i + margin]):
            extended[i] = True
    for i in range(len(is_moving) - 1, margin, -1):
        if np.any(is_moving[i - margin:i]):
            extended[i] = True
    return extended

def integrate_accel_to_vel(accels_xy, time_step, is_moving):
    acc = accels_xy  # already in m/s²
    velocity = np.zeros(2)
    velocities = []

    for a, move in zip(acc, is_moving):
        if move:
            velocity += a * time_step
        velocities.append(velocity.copy())

    return np.array(velocities)

def remove_velocity_drift(timestamps, velocities, is_moving):
    drift_corrected = velocities.copy()
    diff = np.diff(is_moving.astype(int), prepend=0)
    moving_periods = []
    start = None
    for i, change in enumerate(diff):
        if change == 1:
            start = i
        elif change == -1 and start is not None:
            moving_periods.append((start, i))
            start = None

    for start, stop in moving_periods:
        t = [timestamps[start], timestamps[stop]]
        t_new = timestamps[start:stop+1]
        for i in range(2):  # X and Y only
            interp = interp1d(t, [velocities[start, i], velocities[stop, i]], fill_value="extrapolate")
            drift_corrected[start:stop+1, i] -= interp(t_new)
    return drift_corrected

def integrate_vel_to_pos(velocities, time_step):
    position = np.zeros(2)
    positions = []
    for v in velocities:
        position += v * time_step
        positions.append(position.copy())
    return np.array(positions)

def plot_2d_trajectory(positions):
    x, y = positions[:, 0], positions[:, 1]
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    lc = LineCollection(segments, cmap='viridis', linewidth=2)
    lc.set_array(np.linspace(0, 1, len(segments)))

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.add_collection(lc)
    ax.autoscale()
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_title("2D Trajectory (X-Y) with Time Gradient")
    ax.grid(True)
    ax.set_aspect('equal')
    plt.colorbar(lc, ax=ax, label="Normalized Time")
    plt.tight_layout()
    plt.show()

def main():
    full_data = load_sensor_data("../sensor_logs/short_walk.csv")  # adjust path if needed
    time = full_data["Time (s)"].values
    sample_rate, time_step = calculate_sample_rate(time)

    print(f"Sample rate: {sample_rate:.2f} Hz")
    print(f"Time step: {time_step:.6f} s")

    eulers, quaternions = calculate_orientation(full_data, time_step)
    accel = full_data[['Accelerometer X (g)', 'Accelerometer Y (g)', 'Accelerometer Z (g)']].values
    lin_accels_xy = remove_gravity(accel, quaternions)

    is_moving = detect_movement(lin_accels_xy)
    is_moving = apply_margin(is_moving, sample_rate)

    velocities_xy = integrate_accel_to_vel(lin_accels_xy, time_step, is_moving)
    velocities_xy = remove_velocity_drift(time, velocities_xy, is_moving)
    positions_xy = integrate_vel_to_pos(velocities_xy, time_step)

    print("\nFinal estimated X-Y position (m):", positions_xy[-1])
    print("Total distance in XY plane (m):", np.linalg.norm(positions_xy[-1]))

    plot_2d_trajectory(positions_xy)

if __name__ == "__main__":
    main()
