from dataclasses import dataclass
from scipy.signal import detrend
import imufusion
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
import itertools, json, csv, time

# === Load sensor data ===
data = np.genfromtxt("../../optical_flow_method_data/IMU_new/square_constant_orientation.csv",
                     delimiter=",", skip_header=1)
timestamp     = data[:, 0]
gyroscope_raw = data[:, 1:4]
accelerometer = data[:, 4:7]

# === Sample rate ===
fs = 1.0 / np.mean(np.diff(timestamp))
dt = np.diff(timestamp, prepend=timestamp[0])
G  = 9.80665  # gravity m/s²
print("Sample Rate:", fs)

# === Defaults ===
defaults = {
    "gain": 5.0,
    "gyro_range": 250,
    "accel_rej": 2,
    "mag_rej": 0,
    "rej_timeout_mult": 1,
    "motion_threshold": 0.01,
    "smoothing_margin_mult": 1,
}

# === Parameter sweep definitions (full ranges) ===
param_defs = {
    "gain": np.round(np.arange(1.0, 3.5, 0.5), 2),
    "gyro_range": [250],
    "accel_rej": [5, 10, 15],
    "mag_rej": [0],
    "rej_timeout_mult": [2, 4],
    "motion_threshold": np.round(np.arange(0.05, 0.20, 0.05), 2),
    "smoothing_margin_mult": [1, 2],
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
        earth_acc[i] = ahrs.earth_acceleration

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
    return pos, err_xy

# =========================================================
# Build full Cartesian grid
# =========================================================
def build_grid():
    keys = list(param_defs.keys())
    combos = itertools.product(*(param_defs[k] for k in keys))
    grid = []
    for combo in combos:
        p = {k: v for k, v in zip(keys, combo)}
        grid.append(p)
    return grid

GRID = build_grid()
print("Total combos:", len(GRID))

# =========================================================
# UI & Animation
# =========================================================
fig = plt.figure(figsize=(10, 8))
ax = fig.add_axes([0.1, 0.3, 0.75, 0.65])
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
ax.set_aspect("equal", adjustable="datalim")
ax.grid(True)

status = fig.text(0.1, 0.25, "", fontsize=10, family="monospace")
current_index = 0
is_playing = False
timer = fig.canvas.new_timer(interval=50)

# Saved results
saved_rows = []

def params_to_str(params, err):
    return (f"g={params['gain']:.2f}, gr={params['gyro_range']}, "
            f"ar={params['accel_rej']}, mr={params['mag_rej']}, "
            f"rt_mult={params['rej_timeout_mult']}, "
            f"mt={params['motion_threshold']:.2f}, "
            f"sm_mult={params['smoothing_margin_mult']} | errXY={err:.2f} m")

def goto_index(i):
    global current_index
    current_index = max(0, min(len(GRID)-1, i))
    params = GRID[current_index]
    pos, err = run_once(params)

    ax.clear()
    ax.plot(pos[:,0], pos[:,1], lw=1.6)
    ax.plot(pos[0,0], pos[0,1], "ro", markersize=8)   # red start
    ax.plot(pos[-1,0], pos[-1,1], "yo", markersize=8) # yellow end

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title(params_to_str(params, err))
    status.set_text(f"Combo {current_index+1}/{len(GRID)}")
    ax.grid(True)
    fig.canvas.draw_idle()

def step(delta):
    goto_index(current_index + delta)

def on_timer():
    if is_playing:
        step(1)
        if current_index >= len(GRID)-1:
            toggle_play()

timer.add_callback(on_timer)

def toggle_play(event=None):
    global is_playing
    is_playing = not is_playing
    if is_playing:
        timer.start()
    else:
        timer.stop()

def save_current(event=None):
    params = GRID[current_index]
    pos, err = run_once(params)
    row = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        **params,
        "err_xy": err,
    }
    saved_rows.append(row)
    with open("saved_params.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=row.keys())
        writer.writeheader()
        for r in saved_rows:
            writer.writerow(r)
    with open("saved_params.json", "w", encoding="utf-8") as f:
        json.dump(saved_rows, f, indent=2)
    print("Saved current parameters:", row)

# Buttons
ax_prev = fig.add_axes([0.1, 0.15, 0.15, 0.05]); b_prev = Button(ax_prev, "⟵ Back")
ax_play = fig.add_axes([0.3, 0.15, 0.15, 0.05]); b_play = Button(ax_play, "Play/Pause")
ax_next = fig.add_axes([0.5, 0.15, 0.15, 0.05]); b_next = Button(ax_next, "Next ⟶")
ax_save = fig.add_axes([0.7, 0.15, 0.15, 0.05]); b_save = Button(ax_save, "Save Current")

b_prev.on_clicked(lambda e: step(-1))
b_next.on_clicked(lambda e: step(1))
b_play.on_clicked(toggle_play)
b_save.on_clicked(save_current)

# Init
goto_index(0)
plt.show()
