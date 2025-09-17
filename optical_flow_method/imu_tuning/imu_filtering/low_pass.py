import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# === Load data ===
file_path = "../../../optical_flow_method_data/combined_samples/13_09_25_MILC/straight1/download_imu_cropped.csv"
df = pd.read_csv(file_path)

t = df["time"].to_numpy()
dt = np.mean(np.diff(t))

x_data = df["accel x"].to_numpy()
y_data = df["accel y"].to_numpy()
z_data = df["accel z"].to_numpy()

def integrate(data, dt):
    vel = [0.0]
    for i in range(1, len(data)):
        v_next = vel[-1] + 0.5 * (data[i] + data[i-1]) * dt
        vel.append(v_next)
    return np.array(vel)

# === Low-pass filter ===
w_c = 20.0  # rad/s cutoff (adjust as needed)
alpha = (dt * w_c) / (2 + dt * w_c)
gamma = (2 - dt * w_c) / (2 + dt * w_c)

def lowpass_filter(data):
    out = [data[0]]
    for i in range(1, len(data)):
        out.append(alpha * (data[i] + data[i-1]) + gamma * out[-1])
    return np.array(out)

x_f, y_f, z_f = lowpass_filter(x_data), lowpass_filter(y_data), lowpass_filter(z_data)

# Integrate
x_v, y_v, z_v = integrate(x_f, dt), integrate(y_f, dt), integrate(z_f, dt)
x_p, y_p, z_p = integrate(x_v, dt), integrate(y_v, dt), integrate(z_v, dt)

# Plot
fig, axs = plt.subplots(3, 1, figsize=(10, 8))

axs[0].plot(t, x_data, color='lightblue', alpha=0.6, label='Raw X')
axs[0].plot(t, y_data, color='navajowhite', alpha=0.6, label='Raw Y')
axs[0].plot(t, z_data, color='lightgreen', alpha=0.6, label='Raw Z')
axs[0].plot(t, x_f, color='blue', label='Filtered X')
axs[0].plot(t, y_f, color='orange', label='Filtered Y')
axs[0].plot(t, z_f, color='green', label='Filtered Z')
axs[0].set_title("Acceleration (Raw vs Low-pass)")
axs[0].legend()

axs[1].plot(t, x_v, label='Vel X')
axs[1].plot(t, y_v, label='Vel Y')
axs[1].plot(t, z_v, label='Vel Z')
axs[1].set_title("Velocity (Low-pass)")
axs[1].legend()

axs[2].plot(t, x_p, label='Pos X')
axs[2].plot(t, y_p, label='Pos Y')
axs[2].plot(t, z_p, label='Pos Z')
axs[2].set_title("Position (Low-pass)")
axs[2].legend()

plt.tight_layout()
plt.show()