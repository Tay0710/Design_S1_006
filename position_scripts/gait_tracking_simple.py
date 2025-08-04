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

# Instantiate AHRS algorithms
offset = imufusion.Offset(int(sample_rate))
ahrs = imufusion.Ahrs()

# Tuning Variables
gain = 0.5
gyro_range = 500
accel_rej = 10
mag_rej = 0
rej_timeout = 3 * int(sample_rate)
motion_threshold = 0.4
smoothing_margin = int(0.2 * sample_rate)

ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,
                                   gain,
                                   gyro_range,
                                   accel_rej,
                                   mag_rej,
                                   rej_timeout)

# Process sensor data
delta_time = numpy.diff(timestamp, prepend=timestamp[0])

euler = numpy.empty((len(timestamp), 3))
internal_states = numpy.empty((len(timestamp), 3))
acceleration = numpy.empty((len(timestamp), 3))

for index in range(len(timestamp)):
    gyroscope[index] = offset.update(gyroscope[index])
    ahrs.update_no_magnetometer(gyroscope[index], accelerometer[index], delta_time[index])
    euler[index] = ahrs.quaternion.to_euler()
    internal = ahrs.internal_states
    internal_states[index] = numpy.array([
        internal.acceleration_error,
        internal.accelerometer_ignored,
        internal.acceleration_recovery_trigger
    ])
    acceleration[index] = ahrs.earth_acceleration
    if index % 500 == 0:  # Only print every 500 samples to avoid flooding
        print(f"[{index}] accel = {acceleration[index]}, norm = {numpy.linalg.norm(acceleration[index]):.2f}")
    print(f"[{index}] scaled = {acceleration[index]}")


acc_norms = numpy.linalg.norm(acceleration, axis=1)
print(f"\n📊 Acceleration stats:")
print(f"  Mean norm: {numpy.mean(acc_norms):.2f} m/s²")
print(f"  Min norm:  {numpy.min(acc_norms):.2f} m/s²")
print(f"  Max norm:  {numpy.max(acc_norms):.2f} m/s²")
print(f"  Std dev:   {numpy.std(acc_norms):.2f} m/s²")


# Identify moving periods
is_moving = numpy.zeros(len(timestamp), dtype=bool)
for index in range(len(timestamp)):
    acc_norm = numpy.linalg.norm(acceleration[index])
    # if index % 500 == 0:
    #     print(f"[{index}] Acc norm for motion check = {acc_norm:.2f}")
    is_moving[index] = acc_norm > motion_threshold


for index in range(len(timestamp) - smoothing_margin):
    is_moving[index] = numpy.any(is_moving[index:(index + smoothing_margin)])

for index in range(len(timestamp) - 1, smoothing_margin, -1):
    is_moving[index] = numpy.any(is_moving[(index - smoothing_margin):index])

print(f"\n✅ Motion detection summary:")
print(f"Total samples: {len(is_moving)}")
print(f"Moving samples: {numpy.count_nonzero(is_moving)}")
print(f"Still samples: {len(is_moving) - numpy.count_nonzero(is_moving)}\n")

# Calculate velocity
velocity = numpy.zeros((len(timestamp), 3))

for index in range(len(timestamp)):
    if is_moving[index]:
        velocity[index] = velocity[index - 1] + delta_time[index] * acceleration[index]
    else:
        velocity[index] = numpy.zeros(3)  # clamp to zero during still  # carry forward

    # if index % 500 == 0:
    #     print(f"[{index}] velocity = {velocity[index]}, norm = {numpy.linalg.norm(velocity[index]):.3f}, is_moving = {bool(is_moving[index])}")

print("Motion detected in", numpy.count_nonzero(is_moving), "out of", len(is_moving), "samples")

# Find moving periods
is_moving_diff = numpy.diff(is_moving, append=is_moving[-1])

@dataclass
class IsMovingPeriod:
    start_index: int = -1
    stop_index: int = -1

is_moving_periods = []
period = IsMovingPeriod()

for index in range(len(timestamp)):
    if period.start_index == -1 and is_moving_diff[index] == 1:
        period.start_index = index
    elif period.start_index != -1 and is_moving_diff[index] == -1:
        period.stop_index = index
        is_moving_periods.append(period)
        period = IsMovingPeriod()

# Remove integral drift
velocity_drift = numpy.zeros((len(timestamp), 3))
for p in is_moving_periods:
    start, stop = p.start_index, p.stop_index
    t = [timestamp[start], timestamp[stop]]
    for i in range(3):
        velocity_drift[start:stop + 1, i] = interp1d(t, [velocity[start, i], velocity[stop, i]])(timestamp[start:stop + 1])
velocity -= velocity_drift

# Calculate position
position = numpy.zeros((len(timestamp), 3))
for index in range(len(timestamp)):
    position[index] = position[index - 1] + delta_time[index] * velocity[index]

# Print final error
print("Error: {:.3f} m".format(numpy.linalg.norm(position[-1])))

fig, axes = pyplot.subplots(nrows=3, figsize=(10, 12))

# === Subplot 1: 2D Trajectory (X vs Y) ===
axes[0].plot(position[:, 0], position[:, 1], marker='o', markersize=1, linewidth=1)
axes[0].set_xlabel("X Position (m)")
axes[0].set_ylabel("Y Position (m)")
axes[0].set_title("2D Trajectory (X-Y Plane)")
axes[0].axis('equal')
axes[0].grid(True)

# === Subplot 2: Velocities over Time ===
axes[1].plot(timestamp, velocity[:, 0], label="Velocity X", linewidth=1)
axes[1].plot(timestamp, velocity[:, 1], label="Velocity Y", linewidth=1)
axes[1].plot(timestamp, velocity[:, 2], label="Velocity Z", linewidth=1)
axes[1].set_xlabel("Time (s)")
axes[1].set_ylabel("Velocity (m/s)")
axes[1].set_title("Velocity Components Over Time")
axes[1].legend()
axes[1].grid(True)

# === Subplot 3: Accelerations over Time ===
axes[2].plot(timestamp, acceleration[:, 0], label="Acceleration X", linewidth=1)
axes[2].plot(timestamp, acceleration[:, 1], label="Acceleration Y", linewidth=1)
axes[2].plot(timestamp, acceleration[:, 2], label="Acceleration Z", linewidth=1)
axes[2].set_xlabel("Time (s)")
axes[2].set_ylabel("Acceleration (m/s²)")
axes[2].set_title("Acceleration Components Over Time")
axes[2].legend()
axes[2].grid(True)

# === Plot is_moving mask on velocity plot ===
axes[1].fill_between(timestamp, -5, 5, where=is_moving, color='orange', alpha=0.2, label='Motion Detected')
axes[1].legend()

# Add extra space between subplots
pyplot.tight_layout(pad=4.0)
pyplot.show()
