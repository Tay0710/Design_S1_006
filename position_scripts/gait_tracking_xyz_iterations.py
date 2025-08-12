from dataclasses import dataclass
from scipy.signal import detrend
import imufusion
import matplotlib.pyplot as pyplot
import numpy

# === Import sensor data ===
data = numpy.genfromtxt("../sensor_logs/ICM20948/2025-08-08 19-14-49.csv", delimiter=",", skip_header=1)

timestamp     = data[:, 0]
gyroscope     = data[:, 1:4]
accelerometer = data[:, 4:7]

# === Calculate sample rate ===
sample_rate = 1.0 / numpy.mean(numpy.diff(timestamp))
print("Sample Rate: ", sample_rate)

# === Instantiate AHRS algorithms ===
offset = imufusion.Offset(int(sample_rate))
ahrs   = imufusion.Ahrs()

# === Tuning Variables (except motion_threshold we will sweep) ===
gain = 0.5
gyro_range = 2000
accel_rej = 5
mag_rej = 0
rej_timeout = 3 * int(sample_rate)
smoothing_margin = int(0.2 * sample_rate)

ahrs.settings = imufusion.Settings(
    imufusion.CONVENTION_NWU,
    gain,
    gyro_range,
    accel_rej,
    mag_rej,
    rej_timeout
)

# === Gravity removal constants ===
G = 9.80665
GRAVITY_G_NWU = numpy.array([0.0, 0.0, -1.0])  # NWU: Up=+Z

# === Precompute orientation and linear acceleration (m/s^2) once ===
delta_time   = numpy.diff(timestamp, prepend=timestamp[0])
acceleration = numpy.empty((len(timestamp), 3))  # linear accel (m/s^2)

for i in range(len(timestamp)):
    gyroscope[i] = offset.update(gyroscope[i])
    accel_g = accelerometer[i] / G                     # imufusion expects g
    ahrs.update_no_magnetometer(gyroscope[i], accel_g, delta_time[i])
    earth_accel_g = ahrs.earth_acceleration           # in g, Earth frame
    acceleration[i] = (earth_accel_g - GRAVITY_G_NWU) * G  # -> m/s^2

# Quick stats
acc_norms = numpy.linalg.norm(acceleration, axis=1)
print(f"\n📊 Linear acceleration stats (m/s²): mean={numpy.mean(acc_norms):.3f}, "
      f"min={numpy.min(acc_norms):.3f}, max={numpy.max(acc_norms):.3f}, std={numpy.std(acc_norms):.3f}")

# === Helper: compute position for a given motion_threshold ===
def compute_position_for_threshold(threshold: float):
    is_moving = numpy.linalg.norm(acceleration, axis=1) > threshold

    # Smooth mask
    for idx in range(len(timestamp) - smoothing_margin):
        is_moving[idx] = numpy.any(is_moving[idx:(idx + smoothing_margin)])
    for idx in range(len(timestamp) - 1, smoothing_margin, -1):
        is_moving[idx] = numpy.any(is_moving[(idx - smoothing_margin):idx])

    velocity = numpy.zeros((len(timestamp), 3))
    for i in range(1, len(timestamp)):
        if is_moving[i]:
            velocity[i] = velocity[i - 1] + delta_time[i] * acceleration[i]
        else:
            velocity[i] = numpy.zeros(3)

    for k in range(3):
        velocity[:, k] = detrend(velocity[:, k], type='linear')

    position = numpy.zeros((len(timestamp), 3))
    for i in range(1, len(timestamp)):
        position[i] = position[i - 1] + delta_time[i] * velocity[i]

    moving_count = int(numpy.count_nonzero(is_moving))
    error = float(numpy.linalg.norm(position[-1]))
    return position, moving_count, error

# === Sweep thresholds ===
thresholds = numpy.round(numpy.arange(0.01, 2.01, 0.01), 2)
positions   = []
moving_cts  = []
errors      = []

for th in thresholds:
    pos, mcnt, err = compute_position_for_threshold(th)
    positions.append(pos)
    moving_cts.append(mcnt)
    errors.append(err)
    print(f"th={th:.2f} → moving={mcnt:5d}, error={err:.3f} m")

# === Paged, scrollable viewer (4 subplots per page) ===
PAGE_SIZE = 4
num_pages = int(numpy.ceil(len(thresholds) / PAGE_SIZE))
page_idx  = 0

fig, axes = pyplot.subplots(PAGE_SIZE, 1, figsize=(9, 3 * PAGE_SIZE), squeeze=False)
axes = axes.ravel()

def draw_page(page):
    start = page * PAGE_SIZE
    end   = min(start + PAGE_SIZE, len(thresholds))
    for ax in axes:
        ax.clear()

    for row, idx in enumerate(range(start, end)):
        ax = axes[row]
        pos = positions[idx]
        th  = thresholds[idx]
        err = errors[idx]
        ax.plot(pos[:, 0], pos[:, 1], linewidth=1)
        ax.plot(pos[0, 0],  pos[0, 1],  'o', markersize=4)  # start
        ax.plot(pos[-1, 0], pos[-1, 1], 'x', markersize=5)  # end
        ax.set_title(f"Threshold {th:.2f} m/s² — Error = {err:.3f} m")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.axis('equal')
        ax.grid(True)

    # hide any unused axes on last page
    for k in range(end - start, PAGE_SIZE):
        axes[k].axis("off")

    fig.suptitle(f"Page {page + 1}/{num_pages}  (Scroll / ← → to change page)", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.canvas.draw_idle()

def on_scroll(event):
    global page_idx
    if event.button == 'up':
        page_idx = max(0, page_idx - 1)
    elif event.button == 'down':
        page_idx = min(num_pages - 1, page_idx + 1)
    draw_page(page_idx)

def on_key(event):
    global page_idx
    if event.key in ('right', 'down', 'pagedown'):
        page_idx = min(num_pages - 1, page_idx + 1)
        draw_page(page_idx)
    elif event.key in ('left', 'up', 'pageup'):
        page_idx = max(0, page_idx - 1)
        draw_page(page_idx)
    elif event.key in ('home',):
        page_idx = 0
        draw_page(page_idx)
    elif event.key in ('end',):
        page_idx = num_pages - 1
        draw_page(page_idx)

fig.canvas.mpl_connect('scroll_event', on_scroll)
fig.canvas.mpl_connect('key_press_event', on_key)

draw_page(page_idx)
pyplot.show()
