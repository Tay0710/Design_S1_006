import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import imufusion
from scipy.signal import detrend

# === Load sensor data ===
data = np.genfromtxt("../../optical_flow_method_data/combined_samples/square2/IMU_combined_square2.csv",
                     delimiter=",", skip_header=1)

timestamp = data[:, 0]
gyroscope = data[:, 1:4]
accelerometer = data[:, 4:7]

sample_rate = 1.0 / np.mean(np.diff(timestamp))
print("Sample Rate: ", sample_rate)

# === Gravity constant ===
G = 9.80665
GRAVITY_G_NWU = np.array([0.0, 0.0, -1.0])

# === Default tuning parameters (use multipliers for rej_timeout & smoothing_margin) ===
defaults = {
    "gain": 10,
    "gyro_range": 2000,
    "accel_rej": 6,
    "mag_rej": 0,
    "rej_timeout_mult": 3,        # multiplier (not multiplied yet)
    "motion_threshold": 0.1,
    "smoothing_margin_mult": 1,   # multiplier (not multiplied yet)
}

# === Processing pipeline ===
def run_pipeline(gain, gyro_range, accel_rej, mag_rej, rej_timeout_mult, motion_threshold, smoothing_margin_mult):
    # convert multipliers into actual values in samples
    rej_timeout = int(rej_timeout_mult * sample_rate)
    smoothing_margin = int(smoothing_margin_mult * sample_rate)

    offset = imufusion.Offset(int(sample_rate))
    ahrs = imufusion.Ahrs()
    ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,
                                       gain, gyro_range,
                                       accel_rej, mag_rej,
                                       rej_timeout)

    delta_time = np.diff(timestamp, prepend=timestamp[0])
    acceleration = np.empty((len(timestamp), 3))

    # Process IMU data
    for index in range(len(timestamp)):
        gyroscope[index] = offset.update(gyroscope[index])
        accel_g = accelerometer[index]  # convert m/s² → g
        ahrs.update_no_magnetometer(gyroscope[index], accel_g, delta_time[index])
        earth_accel_g = ahrs.earth_acceleration
        linear_accel_mps2 = (earth_accel_g) * G
        acceleration[index] = linear_accel_mps2

    # Motion detection
    is_moving = np.zeros(len(timestamp), dtype=bool)
    for index in range(len(timestamp)):
        acc_norm = np.linalg.norm(acceleration[index])
        is_moving[index] = acc_norm > motion_threshold

    for index in range(len(timestamp) - smoothing_margin):
        is_moving[index] = np.any(is_moving[index:(index + smoothing_margin)])
    for index in range(len(timestamp) - 1, smoothing_margin, -1):
        is_moving[index] = np.any(is_moving[(index - smoothing_margin):index])

    # Velocity integration with detrend
    velocity = np.zeros((len(timestamp), 3))
    for index in range(len(timestamp)):
        if is_moving[index]:
            velocity[index] = velocity[index - 1] + delta_time[index] * acceleration[index]
        else:
            velocity[index] = np.zeros(3)
    for i in range(3):
        velocity[:, i] = detrend(velocity[:, i], type='linear')

    # Position integration
    position = np.zeros((len(timestamp), 3))
    for index in range(len(timestamp)):
        position[index] = position[index - 1] + delta_time[index] * velocity[index]

    return position

# === Initial run ===
position = run_pipeline(**defaults)

# === Plot setup ===
fig, ax = plt.subplots(figsize=(6, 6))
plt.subplots_adjust(left=0.25, bottom=0.45)  # leave room for sliders
traj_line, = ax.plot(position[:, 0], position[:, 1], lw=1)
ax.set_title("2D Trajectory (XY)")
ax.set_aspect("equal", adjustable="box")
ax.grid(True)

ax.set_xlim(position[:, 0].min(), position[:, 0].max())
ax.set_ylim(position[:, 1].min(), position[:, 1].max())

# === Create sliders ===
sliders = {}
param_defs = {
    "gain": (0.1, 10.0, defaults["gain"], 0.1),
    "gyro_range": (250, 4000, defaults["gyro_range"], 250),
    "accel_rej": (0, 20, defaults["accel_rej"], 1),
    "mag_rej": (0, 0, defaults["mag_rej"], 1),
    "rej_timeout_mult": (0, 10, defaults["rej_timeout_mult"], 1),       # multiplier
    "motion_threshold": (0, 5, defaults["motion_threshold"], 0.1),
    "smoothing_margin_mult": (1, 5, defaults["smoothing_margin_mult"], 1),  # multiplier
}

for i, (param, (vmin, vmax, vinit, vstep)) in enumerate(param_defs.items()):
    ax_slider = plt.axes([0.25, 0.35 - i * 0.04, 0.65, 0.03])
    slider = Slider(ax_slider, param, vmin, vmax, valinit=vinit, valstep=vstep)
    sliders[param] = slider

# === Update function ===
def update(val):
    params = {p: sliders[p].val for p in sliders}
    position = run_pipeline(**params)
    traj_line.set_xdata(position[:, 0])
    traj_line.set_ydata(position[:, 1])
    ax.set_xlim(position[:, 0].min(), position[:, 0].max())
    ax.set_ylim(position[:, 1].min(), position[:, 1].max())
    fig.canvas.draw_idle()

for s in sliders.values():
    s.on_changed(update)

# === Reset button ===
resetax = plt.axes([0.8, 0.02, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')

def reset(event):
    for p, s in sliders.items():
        s.reset()
button.on_clicked(reset)

plt.show()
