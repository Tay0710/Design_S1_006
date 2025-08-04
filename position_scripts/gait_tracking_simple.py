from dataclasses import dataclass
from matplotlib import animation
from scipy.interpolate import interp1d
import imufusion
import matplotlib.pyplot as pyplot
import numpy

# Import sensor data
data = numpy.genfromtxt("../sensor_logs/2025-07-31 14-08-52.csv", delimiter=",", skip_header=1)

timestamp = data[:, 0]
gyroscope = data[:, 1:4]
accelerometer = data[:, 4:7]

# === Calculate sample rate ===
sample_rate = 1.0 / numpy.mean(numpy.diff(timestamp))
print("Sample Rate: ", sample_rate)

# === Loop settings ===
threshold_range = numpy.arange(0.4, 3.1, 0.1)  # from 0.1 to 3.0 inclusive
position_results = []

for motion_threshold in threshold_range:
    print(f"🔍 Threshold = {motion_threshold:.1f}")
    
    # === Reset everything fresh for each threshold ===
    offset = imufusion.Offset(int(sample_rate))
    ahrs = imufusion.Ahrs()

    gain = 0.5
    gyro_range = 500
    accel_rej = 10
    mag_rej = 0
    rej_timeout = 3 * int(sample_rate)
    smoothing_margin = int(0.2 * sample_rate)

    ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,
                                       gain,
                                       gyro_range,
                                       accel_rej,
                                       mag_rej,
                                       rej_timeout)

    delta_time = numpy.diff(timestamp, prepend=timestamp[0])
    euler = numpy.empty((len(timestamp), 3))
    acceleration = numpy.empty((len(timestamp), 3))

    for index in range(len(timestamp)):
        gyroscope[index] = offset.update(gyroscope[index])
        ahrs.update_no_magnetometer(gyroscope[index], accelerometer[index], delta_time[index])
        euler[index] = ahrs.quaternion.to_euler()
        acceleration[index] = ahrs.earth_acceleration

    # === Motion Detection ===
    is_moving = numpy.linalg.norm(acceleration, axis=1) > motion_threshold
    for i in range(len(is_moving) - smoothing_margin):
        is_moving[i] = numpy.any(is_moving[i:i + smoothing_margin])
    for i in range(len(is_moving) - 1, smoothing_margin, -1):
        is_moving[i] = numpy.any(is_moving[i - smoothing_margin:i])

    # === Velocity Integration ===
    velocity = numpy.zeros((len(timestamp), 3))
    for i in range(1, len(timestamp)):
        if is_moving[i]:
            velocity[i] = velocity[i - 1] + delta_time[i] * acceleration[i]
        else:
            velocity[i] = numpy.zeros(3)

    # === Drift Removal ===
    from dataclasses import dataclass
    @dataclass
    class IsMovingPeriod:
        start_index: int = -1
        stop_index: int = -1

    is_moving_diff = numpy.diff(is_moving, append=is_moving[-1])
    is_moving_periods = []
    period = IsMovingPeriod()

    for i in range(len(timestamp)):
        if period.start_index == -1 and is_moving_diff[i] == 1:
            period.start_index = i
        elif period.start_index != -1 and is_moving_diff[i] == -1:
            period.stop_index = i
            is_moving_periods.append(period)
            period = IsMovingPeriod()

    velocity_drift = numpy.zeros_like(velocity)
    for p in is_moving_periods:
        start, stop = p.start_index, p.stop_index
        t = [timestamp[start], timestamp[stop]]
        for j in range(3):
            velocity_drift[start:stop + 1, j] = interp1d(t, [velocity[start, j], velocity[stop, j]])(timestamp[start:stop + 1])
    velocity -= velocity_drift

    # === Position Integration ===
    position = numpy.zeros((len(timestamp), 3))
    for i in range(1, len(timestamp)):
        position[i] = position[i - 1] + delta_time[i] * velocity[i]

    # Save result
    position_results.append((motion_threshold, position.copy()))

import matplotlib.cm as cm
import matplotlib.colors as colors

# === Set up color map ===
from matplotlib import pyplot

cmap = pyplot.get_cmap("viridis")

norm = colors.Normalize(vmin=threshold_range[0], vmax=threshold_range[-1])

fig, ax = pyplot.subplots(figsize=(10, 8))

# === Plot all XY trajectories with gradient color ===
for threshold, pos in position_results:
    color = cmap(norm(threshold))
    ax.plot(pos[:, 0], pos[:, 1], color=color, linewidth=1)

# Add colorbar associated with this axis
sm = cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])  # Needed to create the colorbar
cbar = fig.colorbar(sm, ax=ax, label="Motion Threshold")

ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_title("Trajectory Comparison Across Motion Thresholds (0.1 → 3.0)")
ax.axis("equal")
ax.grid(True)
fig.tight_layout()
pyplot.show()
