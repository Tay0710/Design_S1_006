import numpy as np
import imufusion
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def load_sensor_data(input_csv):
    """Load IMU CSV with timestamp, gyro, accel."""
    data = np.genfromtxt(input_csv, delimiter=",", skip_header=1)
    timestamp = data[:, 0]
    gyroscope = data[:, 1:4]       # [deg/s]
    accelerometer = data[:, 4:7]   # [g]
    return timestamp, gyroscope, accelerometer


def run_ahrs(timestamp, gyroscope, accelerometer):
    """Run AHRS filter and return rotation matrices (flattened)."""
    sample_rate = int(round(1.0 / np.mean(np.diff(timestamp))))
    print("Sample Rate: ", sample_rate)

    delta_time = np.diff(timestamp, prepend=timestamp[0])

    offset = imufusion.Offset(sample_rate)
    ahrs = imufusion.Ahrs()

    ahrs.settings = imufusion.Settings(
        imufusion.CONVENTION_NWU,
        0.5,   # gain
        250,   # gyroscope range
        2,     # acceleration rejection
        10,    # magnetic rejection
        5 * sample_rate,  # recovery trigger period = 5 seconds
    )

    rot_mats = []
    for i in range(len(timestamp)):
        gyroscope[i] = offset.update(gyroscope[i])
        ahrs.update_no_magnetometer(gyroscope[i], accelerometer[i], delta_time[i])
        R = ahrs.quaternion.to_matrix()
        rot_mats.append(R.flatten())  # store flattened row

    return np.array(rot_mats)


def save_rotation_matrices(rot_mats, output_csv):
    """Save rotation matrices to CSV."""
    header = ",".join([f"r{i}{j}" for i in range(3) for j in range(3)])
    np.savetxt(output_csv, rot_mats, delimiter=",", header=header, comments="")
    print(f"Saved rotation matrices to {output_csv}")


def animate_rotation(rot_mats):
    """Simple 3D animation of basis vectors over time."""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([-1, 1])
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")

    def update(frame):
        ax.cla()
        ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([-1, 1])
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")

        R = rot_mats[frame].reshape(3, 3)
        origin = np.zeros(3)
        ax.quiver(*origin, *R[:, 0], color="r", length=1)
        ax.quiver(*origin, *R[:, 1], color="g", length=1)
        ax.quiver(*origin, *R[:, 2], color="b", length=1)
        ax.set_title(f"Frame {frame}/{len(rot_mats)}")

    ani = FuncAnimation(fig, update, frames=len(rot_mats), interval=50)
    plt.show()
    return ani  # keep a reference so it doesn’t get garbage collected


def main():
    # === File paths ===
    input_csv = "../optical_flow_method_data/combined_samples/square2/IMU_combined_square2.csv"
    output_csv = "../optical_flow_method_data/rotation_matrices.csv"

    # === Pipeline ===
    timestamp, gyro, accel = load_sensor_data(input_csv)
    rot_mats = run_ahrs(timestamp, gyro, accel)
    save_rotation_matrices(rot_mats, output_csv)

    # === Run animation ===
    _ = animate_rotation(rot_mats)


if __name__ == "__main__":
    main()
