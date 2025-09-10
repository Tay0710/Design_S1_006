import numpy as np
import pandas as pd
from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt

def load_and_interpolate(imu_file, flow_file):
    """Load IMU + optical flow data, interpolate flow to IMU timestamps."""
    imu = pd.read_csv(imu_file)
    flow = pd.read_csv(flow_file)

    flow_interp = pd.DataFrame()
    for col in ["v_world_x", "v_world_y", "v_world_z", "pos_world_x", "pos_world_y", "pos_world_z"]:
        flow_interp[col] = np.interp(imu["time"], flow["time (s)"], flow[col])
    flow_interp["time"] = imu["time"]

    return imu, flow_interp

def create_kalman_filter(dt, init_pos, init_vel):
    """Initialize a 6-state Kalman filter [px, py, pz, vx, vy, vz]."""
    kf = KalmanFilter(dim_x=6, dim_z=6)

    # State transition (constant velocity model)
    kf.F = np.array([
        [1, 0, 0, dt, 0, 0],
        [0, 1, 0, 0, dt, 0],
        [0, 0, 1, 0, 0, dt],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]
    ])

    # Measurement model: directly observe pos + vel from optical flow
    kf.H = np.eye(6)

    # Initial state (make sure it's (n,1))
    kf.x[:3, 0] = init_pos
    kf.x[3:, 0] = init_vel

    # Covariances (can tune these *******)
    kf.P *= 1.0
    # kf.R = np.diag([0.5]*3 + [0.2]*3)  # measurement noise (flow)
    # kf.Q = np.eye(6) * 1e-2              # process noise (IMU)
    
    kf.R = np.eye(6) * 1e6   # HUGE measurement noise → ignore optical flow
    kf.Q = np.eye(6) * 1e-3

    return kf

def run_filter(kf, imu, flow_interp):
    """Run Kalman filter across dataset."""
    estimates = []
    for i in range(len(imu)):
        # Predict with IMU dynamics
        kf.predict()

        # Update with optical flow measurement
        z = np.hstack([
            flow_interp.loc[i, ["pos_world_x", "pos_world_y", "pos_world_z"]],
            flow_interp.loc[i, ["v_world_x", "v_world_y", "v_world_z"]]
        ])
        kf.update(z)
        estimates.append(kf.x.copy())

    return np.array(estimates)

def save_results(estimates, imu_time, output_file="fused_position.csv"):
    """Save fused results to CSV."""
        # Ensure 2D array is flattened to (N,6)
    estimates = np.squeeze(estimates)
    
    fused = pd.DataFrame({
        "time": imu_time,
        "px": estimates[:, 0],
        "py": estimates[:, 1],
        "pz": estimates[:, 2],
        "vx": estimates[:, 3],
        "vy": estimates[:, 4],
        "vz": estimates[:, 5],
    })
    fused.to_csv(output_file, index=False)
    print(f"Estimation complete → {output_file}")

def plot_results(imu, flow_interp, fused):
    """Plot comparison of IMU, Optical Flow, and Fused estimates."""

    t = imu["time"]

    # === Figure 1: Positions (left) & Velocities (right) ===
    fig, axs = plt.subplots(3, 2, figsize=(14, 10), sharex=True)

    # Mapping for flow column names
    pos_cols = {"x": "pos_world_x", "y": "pos_world_y", "z": "pos_world_z"}
    vel_cols = {"x": "v_world_x", "y": "v_world_y", "z": "v_world_z"}

    # Loop through x, y, z
    for i, axis in enumerate(["x", "y", "z"]):
        # --- Position subplot (left column) ---
        axs[i, 0].plot(t, imu[f"p{axis}"], label="IMU", alpha=0.6)
        axs[i, 0].plot(t, flow_interp[pos_cols[axis]], label="Optical Flow", alpha=0.6)
        axs[i, 0].plot(t, fused[f"p{axis}"], label="Fused", linewidth=2)
        axs[i, 0].set_ylabel(f"p{axis} [m]")
        axs[i, 0].legend(loc="best")

        # --- Velocity subplot (right column) ---
        axs[i, 1].plot(t, imu[f"v{axis}"], label="IMU", alpha=0.6)
        axs[i, 1].plot(t, flow_interp[vel_cols[axis]], label="Optical Flow", alpha=0.6)
        axs[i, 1].plot(t, fused[f"v{axis}"], label="Fused", linewidth=2)
        axs[i, 1].set_ylabel(f"v{axis} [m/s]")
        axs[i, 1].legend(loc="best")

    axs[-1, 0].set_xlabel("Time [s]")
    axs[-1, 1].set_xlabel("Time [s]")

    fig.suptitle("IMU vs Optical Flow vs Fused: Position (left) & Velocity (right)", fontsize=14)
    plt.tight_layout()
    plt.show()

    # === Figure 2: 3D Trajectory ===
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(imu["px"], imu["py"], imu["pz"], label="IMU", alpha=0.6)
    ax.plot(flow_interp["pos_world_x"], flow_interp["pos_world_y"], flow_interp["pos_world_z"],
            label="Optical Flow", alpha=0.6)
    ax.plot(fused["px"], fused["py"], fused["pz"], label="Fused", linewidth=2)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("3D Trajectory Comparison")
    ax.legend()
    plt.show()

def main():
    input_imu_csv = "../optical_flow_method_data/imu_position.csv"
    input_flow_csv = "../optical_flow_method_data/xy_velocities_to_world_frame.csv"
    imu, flow_interp = load_and_interpolate(input_imu_csv, input_flow_csv)

    dt = np.mean(np.diff(imu["time"]))
    init_pos = imu[["px", "py", "pz"]].iloc[0].values
    init_vel = imu[["vx", "vy", "vz"]].iloc[0].values

    kf = create_kalman_filter(dt, init_pos, init_vel)
    estimates = run_filter(kf, imu, flow_interp)
    save_results(estimates, imu["time"])
    
    # Reload fused CSV for plotting
    fused = pd.read_csv("fused_position.csv")
    plot_results(imu, flow_interp, fused)

if __name__ == "__main__":
    main()
