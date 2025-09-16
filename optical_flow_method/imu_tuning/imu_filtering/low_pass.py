import matplotlib.pyplot as plt
import pandas as pd

# Load IMU CSV
file_path = "../../../optical_flow_method_data/combined_samples/13_09_25_MILC/straight2/download_imu.csv"
df = pd.read_csv(file_path)

# Extract accelerometer data
x_data = df["accel x"].to_numpy()
y_data = df["accel y"].to_numpy()
z_data = df["accel z"].to_numpy()

# Sampling interval from time column
timestamps = df["time"].to_numpy()
T_s = float(df["time"].diff().mean())  # seconds

# Filter cutoff frequency (rad/s)
w_c = 3.7

alpha = (T_s * w_c) / (2 + T_s * w_c)
gamma = (2 - T_s * w_c) / (2 + T_s * w_c)

# Initialize outputs
x_out, y_out, z_out = [x_data[0]], [y_data[0]], [z_data[0]]

# Recursive filtering
for i in range(1, len(x_data)):
    x_out.append(alpha * (x_data[i] + x_data[i - 1]) + gamma * x_out[-1])
    y_out.append(alpha * (y_data[i] + y_data[i - 1]) + gamma * y_out[-1])
    z_out.append(alpha * (z_data[i] + z_data[i - 1]) + gamma * z_out[-1])

# Plot raw vs filtered
plt.figure(figsize=(10, 6))
plt.plot(x_data, color='lightblue', linewidth=0.8, label='Raw Accel X')
plt.plot(x_out, color='blue', linewidth=2, label='Filtered Accel X')

plt.plot(y_data, color='navajowhite', linewidth=0.8, label='Raw Accel Y')
plt.plot(y_out, color='orange', linewidth=2, label='Filtered Accel Y')

plt.plot(z_data, color='lightgreen', linewidth=0.8, label='Raw Accel Z')
plt.plot(z_out, color='green', linewidth=2, label='Filtered Accel Z')

plt.legend()
plt.xlabel('Sample')
plt.ylabel('Acceleration (g)')
plt.title('Accelerometer Data: Raw vs Low-Pass Filtered')
plt.tight_layout()
plt.show()
