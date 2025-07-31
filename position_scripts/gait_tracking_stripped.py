from dataclasses import dataclass
from matplotlib import animation
from scipy.interpolate import interp1d
import imufusion
import matplotlib.pyplot as pyplot
import numpy

# Import sensor data
data = numpy.genfromtxt("../sensor_logs/2025-07-30 22-36-45.csv", delimiter=",", skip_header=1)

# sample_rate = 400

timestamp = data[:, 0]
gyroscope = data[:, 1:4]
accelerometer = data[:, 4:7]

######## SET SAMPLE RATE
sample_rate = 1.0 / numpy.mean(numpy.diff(timestamp))
print("Sample Rate: ", sample_rate)

# Instantiate AHRS algorithms
offset = imufusion.Offset(int(sample_rate))
ahrs = imufusion.Ahrs()

ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,
                                   0.3,  # gain
                                   500,  # gyroscope range
                                   5,  # acceleration rejection
                                   0,  # magnetic rejection
                                   3 * int(sample_rate))  # rejection timeout = 5 seconds

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
    acceleration[index] = 9.81 * ahrs.earth_acceleration

# Identify moving periods
is_moving = numpy.empty(len(timestamp))
for index in range(len(timestamp)):
    is_moving[index] = numpy.sqrt(acceleration[index].dot(acceleration[index])) > 3

margin = int(0.1 * sample_rate)
for index in range(len(timestamp) - margin):
    is_moving[index] = any(is_moving[index:(index + margin)])
for index in range(len(timestamp) - 1, margin, -1):
    is_moving[index] = any(is_moving[(index - margin):index])

# Calculate velocity
velocity = numpy.zeros((len(timestamp), 3))
for index in range(len(timestamp)):
    if is_moving[index]:
        velocity[index] = velocity[index - 1] + delta_time[index] * acceleration[index]

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

# === 2D PLOT: X vs Y ===
pyplot.figure(figsize=(8, 6))
pyplot.plot(position[:, 0], position[:, 1], marker='o', markersize=1, linewidth=1)
pyplot.xlabel("X Position (m)")
pyplot.ylabel("Y Position (m)")
pyplot.title("2D Trajectory (X-Y Plane)")
pyplot.axis('equal')
pyplot.grid(True)
pyplot.tight_layout()
pyplot.show()