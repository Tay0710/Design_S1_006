from dataclasses import dataclass
from matplotlib import animation
from scipy.interpolate import interp1d
import imufusion
import matplotlib.pyplot as pyplot
import numpy

# === Load sensor data ===
data = numpy.genfromtxt("../sensor_logs/2025-07-31 14-08-52.csv", delimiter=",", skip_header=1)

timestamp = data[:, 0]
gyroscope = data[:, 1:4]
accelerometer = data[:, 4:7]

# === Calculate sample rate ===
sample_rate = 1.0 / numpy.mean(numpy.diff(timestamp))
print("Sample Rate: ", sample_rate)

# === AHRS Setup ===
offset = imufusion.Offset(int(sample_rate))
ahrs = imufusion.Ahrs()

ahrs.settings = imufusion.Settings(
    imufusion.CONVENTION_NWU,
    0.3,       # gain
    500,       # gyroscope range
    5,         # acceleration rejection
    0,         # magnetic rejection
    3 * int(sample_rate)  # rejection timeout
)

# === Pre-allocate arrays ===
delta_time = numpy.diff(timestamp, prepend=timestamp[0])
euler = numpy.empty((len(timestamp), 3))
internal_states = numpy.empty((len(timestamp), 3))
acceleration = numpy.empty((len(timestamp), 3))

# === Process sensor data ===
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
    
    acceleration[index] = 9.81 * ahrs.earth_acceleration

    if index % 100 == 0:
        print(f"[{index}] Earth Accel: {ahrs.earth_acceleration}")

# === Acceleration stats ===
acc_mag = numpy.linalg.norm(acceleration, axis=1)
print("Acceleration Magnitude Stats: min {:.3f}, max {:.3f}, mean {:.3f}".format(
    numpy.min(acc_mag), numpy.max(acc_mag), numpy.mean(acc_mag)))

# === Identify moving periods (debugging version) ===
motion_threshold = 0.2
is_moving = numpy.zeros(len(timestamp), dtype=bool)

# Print debug info for first 20 samples
print("\n--- Motion Detection Debug (First 20 Samples) ---")
for index in range(len(timestamp)):
    magnitude = numpy.linalg.norm(acceleration[index])
    is_moving[index] = magnitude > motion_threshold
    if index < 20:
        print(f"Index {index}: |accel| = {magnitude:.3f} > {motion_threshold}? {'Yes' if is_moving[index] else 'No'}")

# Smoothing margin
margin = int(0.1 * sample_rate)
for index in range(len(timestamp) - margin):
    is_moving[index] = any(is_moving[index:(index + margin)])
for index in range(len(timestamp) - 1, margin, -1):
    is_moving[index] = any(is_moving[(index - margin):index])

print("Moving Samples:", numpy.count_nonzero(is_moving))
print("Total Samples:", len(is_moving))

# === Calculate velocity ===
velocity = numpy.zeros((len(timestamp), 3))
USE_MOTION_GATE = False  # FORCE integration ON

print("\n--- Velocity Debug (every 500 samples) ---")
for index in range(1, len(timestamp)):
    if not USE_MOTION_GATE or is_moving[index]:
        velocity[index] = velocity[index - 1] + delta_time[index] * acceleration[index]
    if index % 500 == 0:
        print(f"[{index}] Vx: {velocity[index][0]:.5f}, Vy: {velocity[index][1]:.5f}, |V| = {numpy.linalg.norm(velocity[index]):.5f}")

# === Drift correction ===
@dataclass
class IsMovingPeriod:
    start_index: int = -1
    stop_index: int = -1

is_moving_diff = numpy.diff(is_moving, append=is_moving[-1])
is_moving_periods = []
period = IsMovingPeriod()

for index in range(len(timestamp)):
    if period.start_index == -1 and is_moving_diff[index] == 1:
        period.start_index = index
    elif period.start_index != -1 and is_moving_diff[index] == -1:
        period.stop_index = index
        is_moving_periods.append(period)
        period = IsMovingPeriod()

velocity_drift = numpy.zeros((len(timestamp), 3))
for p in is_moving_periods:
    start, stop = p.start_index, p.stop_index
    t = [timestamp[start], timestamp[stop]]
    for i in range(3):
        velocity_drift[start:stop + 1, i] = interp1d(
            t, [velocity[start, i], velocity[stop, i]]
        )(timestamp[start:stop + 1])

# Optional: apply drift correction
# velocity -= velocity_drift

# === Plot only X and Y velocity ===
pyplot.figure()
pyplot.plot(timestamp, velocity[:, 0], label="Vx (X velocity)", color="tab:blue")
pyplot.plot(timestamp, velocity[:, 1], label="Vy (Y velocity)", color="tab:orange")
pyplot.title("Velocity in X and Y Directions")
pyplot.xlabel("Time (s)")
pyplot.ylabel("Velocity (m/s)")
pyplot.grid(True)
pyplot.legend()
pyplot.tight_layout()

# === Integrate position ===
position = numpy.zeros((len(timestamp), 3))
for index in range(1, len(timestamp)):
    position[index] = position[index - 1] + delta_time[index] * velocity[index]

print("\n--- Final Position Stats ---")
print("Error: {:.3f} m".format(numpy.linalg.norm(position[-1])))
print("X range:", numpy.min(position[:, 0]), "to", numpy.max(position[:, 0]))
print("Y range:", numpy.min(position[:, 1]), "to", numpy.max(position[:, 1]))

# === Plot 2D trajectory ===
pyplot.figure(figsize=(8, 6))
pyplot.plot(position[:, 0], position[:, 1], marker='o', markersize=1, linewidth=1)
pyplot.xlabel("X Position (m)")
pyplot.ylabel("Y Position (m)")
pyplot.title("2D Trajectory (X-Y Plane)")
pyplot.axis('equal')
pyplot.grid(True)
pyplot.tight_layout()
pyplot.show()
