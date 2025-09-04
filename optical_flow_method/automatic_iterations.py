from dataclasses import dataclass
from scipy.signal import detrend
import imufusion
import numpy as np
import itertools, json, csv, time

# === Load sensor data ===
data = np.genfromtxt(
    "../optical_flow_method_data/combined_samples/square2/IMU_combined_square2.csv",
    delimiter=",", skip_header=1
)
timestamp     = data[:, 0]
gyroscope_raw = data[:, 1:4]
accelerometer = data[:, 4:7]

# === Sample rate ===
fs = 1.0 / np.mean(np.diff(timestamp))
dt = np.diff(timestamp, prepend=timestamp[0])
G  = 9.80665  # gravity m/s²
print("Sample Rate:", fs)

# === Parameter sweep definitions (full ranges) ===
param_defs = {
    "gain": np.round(np.arange(0.5, 2.0, 0.1), 2),
    "gyro_range": [500, 1000, 1500, 2000],
    "accel_rej": list(range(4, 14, 2)),
    "mag_rej": [0],
    "rej_timeout_mult": [1, 2, 3, 4, 5],
    "motion_threshold": np.round(np.arange(0.1, 0.2, 0.1), 2),
    "smoothing_margin_mult": [1, 2, 3],
}

# =========================================================
# Core compute (pipeline)
# =========================================================
def run_once(params):
    gyro = np.copy(gyroscope_raw)
    accel = np.copy(accelerometer)

    offset = imufusion.Offset(int(fs))
    ahrs   = imufusion.Ahrs()
    ahrs.settings = imufusion.Settings(
        imufusion.CONVENTION_NWU,
        float(params["gain"]),
        int(params["gyro_range"]),
        int(params["accel_rej"]),
        int(params["mag_rej"]),
        int(params["rej_timeout_mult"] * fs),
    )

    earth_acc = np.empty((len(timestamp), 3))
    for i in range(len(timestamp)):
        gyro[i] = offset.update(gyro[i])
        ahrs.update_no_magnetometer(gyro[i], accel[i], dt[i])
        earth_acc[i] = ahrs.earth_acceleration * G

    # Motion detection
    acc_norm = np.linalg.norm(earth_acc, axis=1)
    is_moving = acc_norm > float(params["motion_threshold"])
    sm = int(params["smoothing_margin_mult"] * fs)
    if sm > 0:
        for i in range(0, len(is_moving) - sm):
            if not is_moving[i] and np.any(is_moving[i:i+sm]):
                is_moving[i] = True
        for i in range(len(is_moving) - 1, sm, -1):
            if not is_moving[i] and np.any(is_moving[i-sm:i]):
                is_moving[i] = True

    # Velocity
    vel = np.zeros((len(timestamp), 3))
    for i in range(len(timestamp)):
        if is_moving[i]:
            vel[i] = vel[i-1] + dt[i] * earth_acc[i]
        else:
            vel[i] = 0.0
    for k in range(3):
        vel[:, k] = detrend(vel[:, k], type="linear")

    # Position
    pos = np.zeros((len(timestamp), 3))
    for i in range(len(timestamp)):
        pos[i] = pos[i-1] + dt[i] * vel[i]

    err_xy = float(np.linalg.norm(pos[-1, :2]))
    return err_xy

# =========================================================
# Run all combos and rank
# =========================================================
def build_grid():
    keys = list(param_defs.keys())
    combos = itertools.product(*(param_defs[k] for k in keys))
    for combo in combos:
        yield {k: v for k, v in zip(keys, combo)}

results = []
for i, params in enumerate(build_grid()):
    err = run_once(params)
    results.append((err, params))
    if (i+1) % 100 == 0:
        print(f"Processed {i+1} combos...")

# Sort by error
results.sort(key=lambda x: x[0])

# Show top 10
print("\nTop 10 parameter sets with lowest errXY:")
for rank, (err, params) in enumerate(results[:10], 1):
    print(f"{rank:2d}. errXY={err:.3f} m | {params}")
