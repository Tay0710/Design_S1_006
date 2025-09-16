import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# === Load data ===
file_path = "../../../optical_flow_method_data/combined_samples/13_09_25_MILC/straight2/download_imu.csv"
df = pd.read_csv(file_path)

# Extract accelerometer data
x_data = df["accel x"].to_numpy()
y_data = df["accel y"].to_numpy()
z_data = df["accel z"].to_numpy()

# Estimate sampling interval (s)
timestamps = df["time"].to_numpy()
dt = float(np.mean(np.diff(timestamps)))

# === Kalman Filter definition ===
class Kalman:
    def __init__(self, dt):
        self.dt = dt
        # State: [acceleration, jerk]
        self.A = np.array([[1, dt],
                           [0, 1]])
        self.C = np.array([[1, 0]])   # Measurement matrix
        
        self.Q = np.identity(2) * 1e-5  # Process noise covariance
        self.R = np.identity(1) * 0.1    # Measurement noise covariance
        
        self.P = np.eye(2)              # Initial error covariance
        self.x = np.array([[0], [0]])   # Initial state

    def predict(self):
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, measurement):
        K = (self.P @ self.C.T) / (self.C @ self.P @ self.C.T + self.R)
        self.x = self.x + K * (measurement - self.C @ self.x)
        self.P = (np.eye(2) - K @ self.C) @ self.P

    def get_state(self):
        return self.x[0, 0], self.P  # Estimated acceleration + covariance

# === Initialize filters ===
kalman_x, kalman_y, kalman_z = Kalman(dt), Kalman(dt), Kalman(dt)

x_out, y_out, z_out = [], [], []
p_out = []

# === Run Kalman filter ===
for k in range(len(x_data)):
    for filt, meas, out in zip(
        (kalman_x, kalman_y, kalman_z),
        (x_data[k], y_data[k], z_data[k]),
        (x_out, y_out, z_out)
    ):
        filt.predict()
        filt.update(meas)
        a_est, P = filt.get_state()
        out.append(a_est)
    
    p_out.append(np.trace(kalman_x.P))  # track covariance trace for debugging

# === Plot results ===
plt.figure(figsize=(10, 6))
plt.plot(x_data, color='lightblue', linewidth=0.8, label='Raw Accel X')
plt.plot(x_out, color='blue', linewidth=2, label='Kalman Accel X')

plt.plot(y_data, color='navajowhite', linewidth=0.8, label='Raw Accel Y')
plt.plot(y_out, color='orange', linewidth=2, label='Kalman Accel Y')

plt.plot(z_data, color='lightgreen', linewidth=0.8, label='Raw Accel Z')
plt.plot(z_out, color='green', linewidth=2, label='Kalman Accel Z')

plt.legend()
plt.xlabel('Sample')
plt.ylabel('Acceleration (g)')
plt.title('Accelerometer Data: Raw vs Kalman Filtered')
plt.tight_layout()
plt.show()

# Plot uncertainty trace (just for X-axis covariance trace as an example)
plt.figure(figsize=(8, 4))
plt.plot(p_out)
plt.title('Trace of error covariance matrix (P) - X Axis')
plt.xlabel('Sample')
plt.ylabel('Trace(P)')
plt.show()
