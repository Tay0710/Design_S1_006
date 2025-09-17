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

# === Integration helper ===
def integrate(data, dt):
    vel = [0.0]
    for i in range(1, len(data)):
        v_next = vel[-1] + 0.5 * (data[i] + data[i-1]) * dt
        vel.append(v_next)
    return np.array(vel)

# === Kalman Filter class ===
class Kalman:
    def __init__(self, dt):
        self.dt = dt

        # State transition & observation matrices
        self.A = np.array([[1, dt], [0, 1]])
        self.C = np.array([[1, 0]])

        # Noise covariances
        self.Q = np.identity(2) * 1e-5   # Process noise
        self.R = np.identity(1) * 0.1    # Measurement noise

        # Initial error covariance
        self.P = np.eye(2)

        # Initial state: [accel, jerk]
        self.x = np.array([[0], [0]])

    def predict(self):
        """ Predict the next state and error covariance """
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, measurement):
        """ Update the state estimate with a new measurement """
        K = (self.P @ self.C.T) / (self.C @ self.P @ self.C.T + self.R)
        self.x = self.x + K * (measurement - self.C @ self.x)
        self.P = (np.eye(2) - K @ self.C) @ self.P

    def get_state(self):
        """ Return estimated acceleration and covariance """
        return self.x[0, 0], self.P

# === Initialize filters for X, Y, Z ===
kalman_x, kalman_y, kalman_z = Kalman(dt), Kalman(dt), Kalman(dt)

x_out, y_out, z_out = [], [], []
p_out = []  # Store trace(P) for X axis

# === Run Kalman filters ===
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

    # Track uncertainty for X axis
    p_out.append(np.trace(kalman_x.P))

# === Integrate filtered accel → velocity → position ===
x_v, y_v, z_v = integrate(x_out, dt), integrate(y_out, dt), integrate(z_out, dt)
x_p, y_p, z_p = integrate(x_v, dt), integrate(y_v, dt), integrate(z_v, dt)

# === Plot Acceleration (raw vs filtered) ===
fig, axs = plt.subplots(3, 1, figsize=(10, 8))

axs[0].plot(t, x_data, color='lightblue', alpha=0.6, label='Raw X')
axs[0].plot(t, y_data, color='navajowhite', alpha=0.6, label='Raw Y')
axs[0].plot(t, z_data, color='lightgreen', alpha=0.6, label='Raw Z')
axs[0].plot(t, x_out, color='blue', label='Kalman X')
axs[0].plot(t, y_out, color='orange', label='Kalman Y')
axs[0].plot(t, z_out, color='green', label='Kalman Z')
axs[0].set_title("Acceleration (Raw vs Kalman)")
axs[0].legend()

# === Plot Velocity ===
axs[1].plot(t, x_v, label='Vel X')
axs[1].plot(t, y_v, label='Vel Y')
axs[1].plot(t, z_v, label='Vel Z')
axs[1].set_title("Velocity (Kalman)")
axs[1].legend()

# === Plot Position ===
axs[2].plot(t, x_p, label='Pos X')
axs[2].plot(t, y_p, label='Pos Y')
axs[2].plot(t, z_p, label='Pos Z')
axs[2].set_title("Position (Kalman)")
axs[2].legend()

plt.tight_layout()
plt.show()

# === Plot trace(P) for debugging ===
plt.figure(figsize=(8, 4))
plt.plot(p_out)
plt.title('Trace of error covariance matrix ($P$) - X Axis')
plt.xlabel('Sample')
plt.ylabel('Trace(P)')
plt.show()
