import numpy as np
import imufusion
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

"""
Has been adapted from the https://github.com/xioTechnologies/Fusion Github repo
"""

# Import sensor data
data = np.genfromtxt("../optical_flow_method_data/sensor_data.csv", delimiter=",", skip_header=1)
# data = np.genfromtxt("../optical_flow_method_data/imc45686_data_stationary.csv", delimiter=",", skip_header=1)
sample_rate = 100  # 100 Hz

timestamp = data[:, 0]
gyroscope = data[:, 1:4]       # [deg/s]
accelerometer = data[:, 4:7]   # [g]
magnetometer = data[:, 7:10]   # [uT] (can leave zeros if no mag)

sample_rate = 100  # Hz
delta_time = np.diff(timestamp, prepend=timestamp[0])

# Instantiate algorithms
offset = imufusion.Offset(sample_rate)
ahrs = imufusion.Ahrs()

ahrs.settings = imufusion.Settings(
    imufusion.CONVENTION_NWU,  # convention
    0.5,  # gain
    2000,  # gyroscope range
    10,  # acceleration rejection
    10,  # magnetic rejection
    5 * sample_rate,  # recovery trigger period = 5 seconds
)

# === Run filter and store rotation matrices ===
rot_mats = []

for i in range(len(timestamp)):
    gyroscope[i] = offset.update(gyroscope[i])
    ahrs.update(gyroscope[i], accelerometer[i], magnetometer[i], delta_time[i])
    R = ahrs.quaternion.to_matrix()  # 3x3 rotation matrix
    rot_mats.append(R.flatten())     # flatten to 9 numbers per row

rot_mats = np.array(rot_mats)

# === Save to CSV ===
header = ",".join([f"r{i}{j}" for i in range(3) for j in range(3)])
np.savetxt("../optical_flow_method_data/rotation_matrices.csv", rot_mats, delimiter=",", header=header, comments="")

print("Saved rotation matrices to rotation_matrices.csv")

# === Simple 3D animation ===
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# Draw basis vectors (rotated axes)
quiver = ax.quiver(0, 0, 0, 1, 0, 0, color="r")  # placeholder

def update(frame):
    ax.cla()  # clear entire 3D axis

    # Reset limits/labels every frame (since cla() wipes them)
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    R = rot_mats[frame].reshape(3, 3)
    origin = np.zeros(3)
    x_axis, y_axis, z_axis = R[:, 0], R[:, 1], R[:, 2]
    ax.quiver(*origin, *x_axis, color="r", length=1)
    ax.quiver(*origin, *y_axis, color="g", length=1)
    ax.quiver(*origin, *z_axis, color="b", length=1)
    ax.set_title(f"Frame {frame}/{len(rot_mats)}")

ani = FuncAnimation(fig, update, frames=len(rot_mats), interval=50)
plt.show()
