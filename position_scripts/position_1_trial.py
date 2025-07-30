import pandas as pd
import numpy as np
import imufusion as fus

def load_sensor_data(filepath):
    full_data = pd.read_csv(filepath)
    return full_data

def calculate_sample_rate(time):
    time_step = np.mean(np.diff(time))
    sample_rate = 1.0 / time_step
    return sample_rate, time_step

def calculate_orientation(full_data, time_step):
    deg_to_rad = np.pi / 180.0
    
    gyro_deg = full_data[['Gyroscope X (deg/s)', 'Gyroscope Y (deg/s)', 'Gyroscope Z (deg/s)']].values
    gyro = gyro_deg * deg_to_rad
    accel = full_data[['Accelerometer X (g)', 'Accelerometer Y (g)', 'Accelerometer Z (g)']].values
    mag = full_data[['Magnetometer X (uT)', 'Magnetometer Y (uT)', 'Magnetometer Z (uT)']].values
    
    ahrs = fus.Ahrs()
    
    euler_angles = []
    quaternions = []
    
    for g, a, m in zip(gyro, accel, mag):
        ahrs.update(g, a, m, time_step)
        
        quaternion = fus.Quaternion(
            np.array([ahrs.quaternion.w,
                    ahrs.quaternion.x,
                    ahrs.quaternion.y,
                    ahrs.quaternion.z])
        )
                
        quaternions.append(quaternion)
        # roll, pitch, yaw
        euler_angles.append(quaternion.to_euler())

    # don't love the np part, find another way
    return np.array(euler_angles), quaternions

def rotate_vector_by_quaternion(v, q):
    # w is a scalar and x. y. z are vector components
    w, qx, qy, qz = q.w, q.x, q.y, q.z
    q_vec = np.array([qx, qy, qz])
    # First cross product: first rotation
    cross_1 = np.cross(q_vec, v)
    # Second cross product: secondary rotation
    cross_2 = np.cross(q_vec, cross_1)
    # Final rotated vector
    rotated_gravity = v + 2 * (w * cross_1 + cross_2)
    return rotated_gravity

def remove_gravity(acceleration_data, quaternions):
    linear_accelerations = []
    gravity_global = np.array([0, 0, 1])
    
    for a, q in zip(acceleration_data, quaternions):
        rotated_gravity = rotate_vector_by_quaternion(gravity_global, q)
        lin_accel = a - rotated_gravity
        linear_accelerations.append(lin_accel)
    return linear_accelerations
    
def main():
    full_data = load_sensor_data("../sensor_logs/sensor_data.csv")
    
    time = full_data["Time (s)"].values
    sample_rate, time_step = calculate_sample_rate(time)
    
    print(f"Sample rate: {sample_rate:.2f} Hz")
    print(f"Time step: {time_step:.6f} s")
    
    eulers, quaternions = calculate_orientation(full_data, time_step)
    print("\nFirst 5 Euler angles (roll, pitch, yaw in radians):")
    print(eulers[:5])
    
    acceleration = full_data[['Accelerometer X (g)', 'Accelerometer Y (g)', 'Accelerometer Z (g)']].values
    lin_accels = remove_gravity(acceleration, quaternions)
    
    print("\nFirst 5 linear accelerations (gravity removed):")
    for i in range(5):
        print(f"{lin_accels[i]}")
    
if __name__ == "__main__":
    main()
