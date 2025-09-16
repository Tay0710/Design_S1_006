import matplotlib.pyplot as plt
import csv
import numpy as np

def running_average(data, window_size):
    return [sum(data[i:i+window_size]) / window_size for i in range(len(data) - window_size + 1)]

def integrate(data, dt):
    vel = [0.0]
    for i in range(1, len(data)):
        v_next = vel[-1] + 0.5 * (data[i] + data[i-1]) * dt
        vel.append(v_next)
    return np.array(vel)

# === Load IMU accel data ===
x_data, y_data, z_data, time = [], [], [], []
file_path = "../../../optical_flow_method_data/combined_samples/rectangle/IMU_combined_rectangle.csv"

with open(file_path, 'r') as file:
    reader = csv.reader(file)
    next(reader)
    for row in reader:
        time.append(float(row[0]))
        x_data.append(float(row[4]))
        y_data.append(float(row[5]))
        z_data.append(float(row[6]))

dt = np.mean(np.diff(time))

# === Running average filter ===
window_size = 10
x_f = running_average(x_data, window_size)
y_f = running_average(y_data, window_size)
z_f = running_average(z_data, window_size)

t_f = time[window_size-1:]

# === Integrate ===
x_v, y_v, z_v = integrate(x_f, dt), integrate(y_f, dt), integrate(z_f, dt)
x_p, y_p, z_p = integrate(x_v, dt), integrate(y_v, dt), integrate(z_v, dt)

# === Plot ===
fig, axs = plt.subplots(3, 1, figsize=(10, 8))

# Acceleration comparison
axs[0].plot(time, x_data, color='lightblue', alpha=0.6, label='Raw X')
axs[0].plot(time, y_data, color='navajowhite', alpha=0.6, label='Raw Y')
axs[0].plot(time, z_data, color='lightgreen', alpha=0.6, label='Raw Z')
axs[0].plot(t_f, x_f, color='blue', label='Filtered X')
axs[0].plot(t_f, y_f, color='orange', label='Filtered Y')
axs[0].plot(t_f, z_f, color='green', label='Filtered Z')
axs[0].set_title("Acceleration (Raw vs Running Avg)")
axs[0].legend()

# Velocity
axs[1].plot(t_f, x_v, label='Vel X')
axs[1].plot(t_f, y_v, label='Vel Y')
axs[1].plot(t_f, z_v, label='Vel Z')
axs[1].set_title("Velocity (Running Avg)")
axs[1].legend()

# Position
axs[2].plot(t_f, x_p, label='Pos X')
axs[2].plot(t_f, y_p, label='Pos Y')
axs[2].plot(t_f, z_p, label='Pos Z')
axs[2].set_title("Position (Running Avg)")
axs[2].legend()

plt.tight_layout()
plt.show()
