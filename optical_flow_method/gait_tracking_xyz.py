from dataclasses import dataclass
from matplotlib import animation
from scipy.interpolate import interp1d
from scipy.signal import detrend
import imufusion
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
import numpy

# === Import sensor data ===
data = numpy.genfromtxt("../optical_flow_method_data/icm456_still_2_15625.csv", delimiter=",", skip_header=1)

time_increments = data[:, 0]
timestamp = numpy.cumsum(time_increments)
gyroscope = data[:, 1:4]
accelerometer = data[:, 4:7]

# === Calculate sample rate ===
sample_rate = 1.0 / numpy.mean(numpy.diff(timestamp))
print("Sample Rate: ", sample_rate)

# === Instantiate AHRS algorithms ===
offset = imufusion.Offset(int(sample_rate))
ahrs = imufusion.Ahrs()

# === Tuning Variables ===
gain = 0.5
gyro_range = 2000
accel_rej = 14
mag_rej = 0
rej_timeout = 3 * int(sample_rate)
motion_threshold = 0.001
smoothing_margin = int(0.2 * sample_rate)

ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,
                                   gain,
                                   gyro_range,
                                   accel_rej,
                                   mag_rej,
                                   rej_timeout)

# === Gravity removal constants ===
G = 9.80665
GRAVITY_G_NWU = numpy.array([0.0, 0.0, -1.0])  # NWU: Up=+Z

# === Process sensor data ===
delta_time = numpy.diff(timestamp, prepend=timestamp[0])

euler = numpy.empty((len(timestamp), 3))
internal_states = numpy.empty((len(timestamp), 3))
acceleration = numpy.empty((len(timestamp), 3))

for index in range(len(timestamp)):
    gyroscope[index] = offset.update(gyroscope[index])

    # Convert accel to g for imufusion
    accel_g = accelerometer[index] / G
    ahrs.update_no_magnetometer(gyroscope[index], accel_g, delta_time[index])

    euler[index] = ahrs.quaternion.to_euler()
    internal = ahrs.internal_states
    internal_states[index] = numpy.array([
        internal.acceleration_error,
        internal.accelerometer_ignored,
        internal.acceleration_recovery_trigger
    ])

    # Gravity removal: earth_acceleration is in g → subtract gravity vector → m/s²
    earth_accel_g = ahrs.earth_acceleration
    linear_accel_mps2 = (earth_accel_g - GRAVITY_G_NWU) * G
    acceleration[index] = linear_accel_mps2

    if index % 500 == 0:
        print(f"[{index}] lin_acc (m/s^2) = {acceleration[index]}, norm = {numpy.linalg.norm(acceleration[index]):.2f}")

# === Acceleration statistics ===
acc_norms = numpy.linalg.norm(acceleration, axis=1)
print(f"\n📊 Acceleration stats:")
print(f"  Mean norm: {numpy.mean(acc_norms):.2f} m/s²")
print(f"  Min norm:  {numpy.min(acc_norms):.2f} m/s²")
print(f"  Max norm:  {numpy.max(acc_norms):.2f} m/s²")
print(f"  Std dev:   {numpy.std(acc_norms):.2f} m/s²")

# === Identify moving periods ===
is_moving = numpy.zeros(len(timestamp), dtype=bool)
for index in range(len(timestamp)):
    acc_norm = numpy.linalg.norm(acceleration[index])
    is_moving[index] = acc_norm > motion_threshold

# Smooth motion detection mask
for index in range(len(timestamp) - smoothing_margin):
    is_moving[index] = numpy.any(is_moving[index:(index + smoothing_margin)])
for index in range(len(timestamp) - 1, smoothing_margin, -1):
    is_moving[index] = numpy.any(is_moving[(index - smoothing_margin):index])

print(f"\n✅ Motion detection summary:")
print(f"Total samples: {len(is_moving)}")
print(f"Moving samples: {numpy.count_nonzero(is_moving)}")
print(f"Still samples: {len(is_moving) - numpy.count_nonzero(is_moving)}\n")

# === Calculate velocity ===
velocity = numpy.zeros((len(timestamp), 3))
for index in range(len(timestamp)):
    if is_moving[index]:
        velocity[index] = velocity[index - 1] + delta_time[index] * acceleration[index]
    else:
        velocity[index] = numpy.zeros(3)  # clamp to zero during still

# === Find moving periods ===
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

# === Remove integral drift from velocity ===
for i in range(3):
    velocity[:, i] = detrend(velocity[:, i], type='linear')

# === Calculate position ===
position = numpy.zeros((len(timestamp), 3))
for index in range(len(timestamp)):
    position[index] = position[index - 1] + delta_time[index] * velocity[index]

# === Print final position error ===
print("Error: {:.3f} m".format(numpy.linalg.norm(position[-1])))

# === Plot results ===
fig = pyplot.figure(figsize=(12, 16))

# Subplot 1: 3D Trajectory
ax3d = fig.add_subplot(311, projection='3d')
ax3d.plot(position[:, 0], position[:, 1], position[:, 2], marker='o', markersize=1, linewidth=1)
ax3d.set_xlabel("X Position (m)")
ax3d.set_ylabel("Y Position (m)")
ax3d.set_zlabel("Z Position (m)")
ax3d.set_title("3D Trajectory")
ax3d.grid(True)

# Subplot 2: Velocities
ax_vel = fig.add_subplot(312)
ax_vel.plot(timestamp, velocity[:, 0], label="Velocity X", linewidth=1)
ax_vel.plot(timestamp, velocity[:, 1], label="Velocity Y", linewidth=1)
ax_vel.plot(timestamp, velocity[:, 2], label="Velocity Z", linewidth=1)
ax_vel.fill_between(timestamp, -5, 5, where=is_moving, color='orange', alpha=0.2, label='Motion Detected')
ax_vel.set_xlabel("Time (s)")
ax_vel.set_ylabel("Velocity (m/s)")
ax_vel.set_title("Velocity Components Over Time")
ax_vel.legend()
ax_vel.grid(True)

# Subplot 3: Accelerations
ax_acc = fig.add_subplot(313)
ax_acc.plot(timestamp, acceleration[:, 0], label="Acceleration X", linewidth=1)
ax_acc.plot(timestamp, acceleration[:, 1], label="Acceleration Y", linewidth=1)
ax_acc.plot(timestamp, acceleration[:, 2], label="Acceleration Z", linewidth=1)
ax_acc.set_xlabel("Time (s)")
ax_acc.set_ylabel("Acceleration (m/s²)")
ax_acc.set_title("Acceleration Components Over Time")
ax_acc.legend()
ax_acc.grid(True)

pyplot.tight_layout(pad=4.0)
pyplot.show()

# === 3D Animation ===
fig_anim = pyplot.figure(figsize=(8, 8))
ax_anim = fig_anim.add_subplot(111, projection='3d')
line_anim, = ax_anim.plot([], [], [], lw=1, marker='o', markersize=1)

ax_anim.set_xlim(numpy.min(position[:, 0]) - 1, numpy.max(position[:, 0]) + 1)
ax_anim.set_ylim(numpy.min(position[:, 1]) - 1, numpy.max(position[:, 1]) + 1)
ax_anim.set_zlim(numpy.min(position[:, 2]) - 1, numpy.max(position[:, 2]) + 1)
ax_anim.set_title("Live 3D Trajectory")
ax_anim.set_xlabel("X Position (m)")
ax_anim.set_ylabel("Y Position (m)")
ax_anim.set_zlabel("Z Position (m)")
ax_anim.grid(True)

def update_3d(frame):
    line_anim.set_data(position[:frame, 0], position[:frame, 1])
    line_anim.set_3d_properties(position[:frame, 2])
    return line_anim,

ani3d = animation.FuncAnimation(fig_anim, update_3d, frames=range(1, len(position), 10), interval=20, blit=True)
pyplot.show()
