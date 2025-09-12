"""
x4_xy_velocity_calculation.py
-----------------------------
Computes drone planar velocities (v_x, v_y) from optical flow angular rates 
and interpolated ToF heights, then integrates to positions.

Overview:
    Optical flow provides angular rates (ωx, ωy) in rad/s. To convert these
    into linear velocities in the horizontal plane, we multiply by the
    estimated drone altitude (h):

        v_x = ωx × h
        v_y = ωy × h

    where h is obtained from ToF sensor data, interpolated to optical 
    flow timestamps.

    Velocities are then integrated using the trapezoidal rule to obtain
    positions (pos_x, pos_y).

Notes:
    - Heights are converted from mm to m before producing velocities.
    - A simple timestamp check is applied: rows are processed only if
      rounded time values match to 6 decimal places.

Inputs:
    - optical_flow_angular_rates.csv
        Columns:
            time (s), wx (rad/s), wy (rad/s)
    - ToF_heights_interp.csv
        Columns:
            time (s), height (mm)

Outputs:
    - xy_velocities.csv
        Columns:
            time (s), v_x (m/s), v_y (m/s),
            pos_x (m), pos_y (m)
"""

import csv
import matplotlib.pyplot as plt
import numpy as np

def velocity_calc(wx, wy, h):
    v_x = wx * h
    v_y = wy * h
    return v_x, v_y

def integrate_positions(times, vx_list, vy_list):
    """Integrate velocities into positions using trapezoidal rule."""
    pos_x = np.zeros(len(times))
    pos_y = np.zeros(len(times))

    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        pos_x[i] = pos_x[i-1] + 0.5 * dt * (vx_list[i] + vx_list[i-1])
        pos_y[i] = pos_y[i-1] + 0.5 * dt * (vy_list[i] + vy_list[i-1])

    return pos_x, pos_y

def main():
    angular_rate_path = "../optical_flow_method_data/optical_flow_angular_rates.csv"
    height_path = "../optical_flow_method_data/ToF_heights_interp.csv"
    output_path = "../optical_flow_method_data/xy_velocities.csv"
    
    times = []
    vx_list = []
    vy_list = []

    with open(angular_rate_path, "r") as f_angular, \
        open(height_path, "r") as f_height, \
        open(output_path, "w", newline="") as f_out:
        
        reader_angular = csv.DictReader(f_angular)
        reader_height = csv.DictReader(f_height)
        writer = csv.writer(f_out)
        writer.writerow(["time (s)", "v_x (m/s)", "v_y (m/s)", "pos_x (m)", "pos_y (m)"])
        
        # First read all values into lists
        angular_rows = list(reader_angular)
        height_rows = list(reader_height)
        
        for row_a, row_h in zip(angular_rows, height_rows):
            # read in both timestamps
            t_a = float(row_a["time (s)"])
            t_h = float(row_h["time"])

            # only proceed if timestamps match (allow noise for now)
            if round(t_a, 6) != round(t_h, 6):
                continue
            
            t = t_a
            wx = float(row_a["wx (rad/s)"])
            wy = float(row_a["wy (rad/s)"])
            h  = float(row_h["height"])
            
            v_x, v_y = velocity_calc(wx, wy, h)
            
            # convert mm/s → m/s
            v_x /= 1000.0
            v_y /= 1000.0

            times.append(t)
            vx_list.append(v_x)
            vy_list.append(v_y)

        # Integrate to positions
        pos_x, pos_y = integrate_positions(np.array(times), np.array(vx_list), np.array(vy_list))

        # Write results with positions
        for t, vx, vy, px, py in zip(times, vx_list, vy_list, pos_x, pos_y):
            writer.writerow([f"{t:.6f}", f"{vx:.6f}", f"{vy:.6f}", f"{px:.6f}", f"{py:.6f}"])
        
        print(f"Wrote {len(times)} rows to {output_path}")    

    # === Plot velocities ===
    plt.figure(figsize=(12, 5))
    plt.subplot(1, 2, 1)
    plt.plot(times, vx_list, label="v_x (m/s)", color="blue")
    plt.plot(times, vy_list, label="v_y (m/s)", color="red")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Optical Flow Derived Velocities")
    plt.legend()
    plt.grid(True)

    # === Plot trajectory (XY) ===
    plt.subplot(1, 2, 2)
    plt.plot(pos_x, pos_y, "-o", markersize=2)
    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.title("Integrated XY Position")
    plt.axis("equal")
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
