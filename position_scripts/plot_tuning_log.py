import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import detrend
import imufusion

# === Load CSV with saved tuning results ===
log_df = pd.read_csv("tuning_log.csv")

# === Load sensor data ===
data = np.genfromtxt("../sensor_logs/2025-08-08 19-14-49.csv", delimiter=",", skip_header=1)
timestamp     = data[:, 0]
gyroscope_raw = data[:, 1:4]
accelerometer = data[:, 4:7]

# === Sample rate ===
fs = 1.0 / np.mean(np.diff(timestamp))
dt = np.diff(timestamp, prepend=timestamp[0])

def run_once(row):
    """Re-run processing for a given CSV row"""
    gyro = np.copy(gyroscope_raw)
    accel = np.copy(accelerometer)

    offset = imufusion.Offset(int(fs))
    ahrs   = imufusion.Ahrs()
    ahrs.settings = imufusion.Settings(
        imufusion.CONVENTION_NWU,
        float(row["gain"]),
        int(row["gyro_range"]),
        int(row["accel_rej"]),
        int(row["mag_rej"]),
        int(row["rej_timeout_samples"]),
    )

    earth_acc = np.empty((len(timestamp), 3))
    for i in range(len(timestamp)):
        gyro[i] = offset.update(gyro[i])
        ahrs.update_no_magnetometer(gyro[i], accel[i], dt[i])
        earth_acc[i] = ahrs.earth_acceleration

    acc_norm = np.linalg.norm(earth_acc, axis=1)
    is_moving = acc_norm > float(row["motion_threshold"])
    sm = int(row["smoothing_margin_samples"])
    if sm > 0:
        for i in range(0, len(is_moving) - sm):
            if not is_moving[i] and np.any(is_moving[i:i+sm]):
                is_moving[i] = True
        for i in range(len(is_moving) - 1, sm, -1):
            if not is_moving[i] and np.any(is_moving[i-sm:i]):
                is_moving[i] = True

    vel = np.zeros((len(timestamp), 3))
    for i in range(len(timestamp)):
        if is_moving[i]:
            vel[i] = vel[i-1] + dt[i] * earth_acc[i]
        else:
            vel[i] = 0.0

    for k in range(3):
        vel[:, k] = detrend(vel[:, k], type='linear')

    pos = np.zeros((len(timestamp), 3))
    for i in range(len(timestamp)):
        pos[i] = pos[i-1] + dt[i] * vel[i]

    return pos

# === Plot in 5×4 subplots ===
fig, axes = plt.subplots(5, 4, figsize=(18, 20))
axes = axes.flatten()

for idx, (ax, (_, row)) in enumerate(zip(axes, log_df.iterrows())):
    pos = run_once(row)
    ax.plot(pos[:,0], pos[:,1], lw=1.5)
    ax.set_title(f"{row['timestamp']}\nerrXY={row['err_xy']:.3f} m", fontsize=9)
    ax.axis('equal')
    ax.grid(True)

# Hide any unused subplots
for j in range(len(log_df), len(axes)):
    fig.delaxes(axes[j])

plt.tight_layout()
plt.show()
