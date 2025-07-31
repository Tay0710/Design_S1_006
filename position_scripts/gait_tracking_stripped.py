from dataclasses import dataclass
from matplotlib import animation
from scipy.interpolate import interp1d
import imufusion
import matplotlib.pyplot as pyplot
import numpy

# Import sensor data
data = numpy.genfromtxt("../sensor_logs/rectangle.csv", delimiter=",", skip_header=1)

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

# Identify moving periods and compare across thresholds
thresholds = [1.0, 2.0, 2.5, 3.0, 4.0, 4.5, 5.0, 5.5, 6.0]
positions = {}
errors = {}

@dataclass
class IsMovingPeriod:
    start_index: int = -1
    stop_index: int = -1

for threshold in thresholds:
    is_moving = numpy.linalg.norm(acceleration, axis=1) > threshold

    # Add margin
    margin = int(0.1 * sample_rate)
    for i in range(len(timestamp) - margin):
        is_moving[i] = any(is_moving[i:i + margin])
    for i in range(len(timestamp) - 1, margin, -1):
        is_moving[i] = any(is_moving[i - margin:i])

    # Calculate velocity
    velocity = numpy.zeros((len(timestamp), 3))
    for index in range(len(timestamp)):
        if is_moving[index]:
            velocity[index] = velocity[index - 1] + delta_time[index] * acceleration[index]

    # Find moving periods
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

    # Store results
    positions[threshold] = position
    errors[threshold] = numpy.linalg.norm(position[-1])

# === 2D PLOT: X vs Y for all thresholds ===
pyplot.figure(figsize=(10, 8))
for threshold in thresholds:
    pos = positions[threshold]
    pyplot.plot(pos[:, 0], pos[:, 1], label=f"Threshold {threshold}")
pyplot.xlabel("X Position (m)")
pyplot.ylabel("Y Position (m)")
pyplot.title("2D Trajectory Overlay (X-Y Plane)")
pyplot.axis('equal')
pyplot.grid(True)
pyplot.legend()
pyplot.tight_layout()

# === Final Position Error vs Threshold ===
pyplot.figure(figsize=(8, 5))
pyplot.bar([str(t) for t in thresholds], [errors[t] for t in thresholds], color=pyplot.cm.viridis(numpy.linspace(0, 1, len(thresholds))))
pyplot.xlabel("Motion Detection Threshold (m/s²)")
pyplot.ylabel("Final Position Error (m)")
pyplot.title("Final Position Error vs Threshold")
pyplot.grid(True)
pyplot.tight_layout()
pyplot.show()
