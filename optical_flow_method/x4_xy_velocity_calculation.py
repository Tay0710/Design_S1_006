"""
x4_xy_velocity_calculation.py
-----------------------------
Computes drone linear velocities (v_x, v_y) from optical-flow angular rates
and interpolated ToF heights, then integrates to positions.

Purpose:
    Compute linear velocities from ωx, ωy and height, then integrate to obtain
    positions. Also derives z-position estimates and finite-difference acceleration.

Overview:
    Uses ωx, ωy (rad/s) and ToF-derived height h to compute linear velocities:
        v_x = ωx × h,  v_y = ωy × h
    Heights are converted from mm to m before the velocity calculation. Velocities are
    integrated with a trapezoidal rule to produce pos_x, pos_y. Z is estimated from
    floor and roof ToF, and accelerations (acc_x, acc_y) are obtained by differentiating
    velocities.

    Notes: Rows are matched by timestamp (rounded to 6 dp).

Usage:
    Called from x0_position_pipeline.py during Stage 4.

Inputs:
    ../optical_flow_method_data/optical_flow_angular_rates.csv
        Columns: time (s), wx (rad/s), wy (rad/s)
    ../optical_flow_method_data/ToF_heights_interp.csv
        Columns: time (s), height (mm)
    ../optical_flow_method_data/ToF_roof_interp.csv
        Columns: time (s), height (mm)

Outputs:
    ../optical_flow_method_data/xy_velocities.csv
        Columns: time (s), v_x (m/s), v_y (m/s), pos_x (m), pos_y (m),
                 pos_z (m), acc_x (m/s^2), acc_y (m/s^2)
"""

import csv
import matplotlib.pyplot as plt
import numpy as np

def velocity_calc(wx, wy, h):
    """Convert angular rates (rad/s) and height (m) to v_x, v_y (m/s)."""
    
    v_x = wx * h
    v_y = wy * h
    
    return v_x, v_y

def integrate_positions(times, vx_list, vy_list):
    """Integrate velocities with trapezoidal rule --> pos_x, pos_y (m)."""
    
    pos_x = np.zeros(len(times-1))
    pos_y = np.zeros(len(times-1))

    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        pos_x[i] = pos_x[i-1] + 0.5 * dt * (vx_list[i] + vx_list[i-1])
        pos_y[i] = pos_y[i-1] + 0.5 * dt * (vy_list[i] + vy_list[i-1])

    return pos_x, pos_y

def differentiate_velocities(times, vx_list, vy_list):
    """Differentiate v_x, v_y --> acc_x, acc_y (m/s^2)."""
    
    acc_x = np.zeros(len(times - 1))
    acc_y = np.zeros(len(times - 1))

    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        acc_x[i] = (vx_list[i] - vx_list[i - 1]) / dt
        acc_y[i] = (vy_list[i] - vy_list[i - 1]) / dt

    return acc_x, acc_y

def find_room_height(roof_rows, height_rows):
    """Average total height over first ~5s (with matched timestamps) and convert to metres."""
    
    sum = 0
    length = 0
    start = 0
    averaging_period = 5

    for row_r, row_h in zip(roof_rows, height_rows):
            t_r = float(row_r["time"])
            t_h = float(row_h["time"])

            # Proceed if timestamps match (allow noise)
            if round(t_r, 6) != round(t_h, 6):
                continue

            if length == 0:
                start = t_r
            
            # Break out of the for loop after the averaging period ends
            if round(t_r, 6) > start + averaging_period:
                break
            
            total_height = float(row_r["height"]) + float(row_h["height"])
            total_height /= 1000

            sum += total_height
            length += 1

    average_height = sum / length
    
    return average_height

def z_position_calc(roof_rows, height_rows, average_height):
    """Compute z from floor, from roof, and a combined estimate (metres)."""
    
    pos_z_h = []
    pos_z_r = []
    pos_z_avg = []
    altitude_thresh = 0.05

    for row_r, row_h in zip(roof_rows, height_rows):
        h_r = average_height - float(row_r["height"]) / 1000
        pos_z_r.append(h_r)

        h_h = float(row_h["height"]) / 1000
        pos_z_h.append(h_h)

        total_height = float(row_r["height"]) + float(row_h["height"])
        if (total_height / 1000 - average_height < -altitude_thresh):
            h_avg = h_r
        else:
            h_avg = (h_h + h_r) / 2
        pos_z_avg.append(h_avg)

    return pos_z_h, pos_z_r, pos_z_avg

def main():
    """Compute velocities, positions, accelerations, and z-estimate. Write CSV and plot."""
    
    angular_rate_path = "../optical_flow_method_data/optical_flow_angular_rates.csv"
    height_path = "../optical_flow_method_data/ToF_heights_interp.csv"
    output_path = "../optical_flow_method_data/xy_velocities.csv"
    roof_path = "../optical_flow_method_data/ToF_roof_interp.csv"
    
    times = []
    vx_list = []
    vy_list = []
    times_2 = []
    total_height_list = []

    with open(angular_rate_path, "r") as f_angular, \
        open(height_path, "r") as f_height, \
        open(roof_path, "r") as f_roof, \
        open(output_path, "w", newline="") as f_out:
        
        reader_angular = csv.DictReader(f_angular)
        reader_height = csv.DictReader(f_height)
        reader_roof = csv.DictReader(f_roof)
        writer = csv.writer(f_out)
        writer.writerow(["time (s)", "v_x (m/s)", "v_y (m/s)", "pos_x (m)", "pos_y (m)", "pos_z (m)", "acc_x (m/s^2)", "acc_y (m/s^2)"])
        
        angular_rows = list(reader_angular)
        height_rows = list(reader_height)
        roof_rows = list(reader_roof)
        
        # Velocities (match times to 6 dp)
        for row_a, row_h in zip(angular_rows, height_rows):
            t_a = float(row_a["time (s)"])
            t_h = float(row_h["time"])

            # Proceed if timestamps match (allow noise)
            if round(t_a, 6) != round(t_h, 6):
                continue
            
            t = t_a
            wx = float(row_a["wx (rad/s)"])
            wy = float(row_a["wy (rad/s)"])
            h  = float(row_h["height"])
            
            v_x, v_y = velocity_calc(wx, wy, h)
            
            v_x /= 1000.0
            v_y /= 1000.0

            v_y = -v_y # Negate y_velocity as OF axis y-axis is opposite to IMU y_axis

            times.append(t)
            vx_list.append(v_x)
            vy_list.append(v_y)

        # Integrate to position
        pos_x, pos_y = integrate_positions(np.array(times), np.array(vx_list), np.array(vy_list))

        # Differentiate to acceleration
        acc_x, acc_y = differentiate_velocities(np.array(times), np.array(vx_list), np.array(vy_list))

        # Total height
        for row_r, row_h in zip(roof_rows, height_rows):
            # read in both timestamps
            t_r = float(row_r["time"])
            t_h = float(row_h["time"])

            # Proceed if timestamps match (allow noise)
            if round(t_r, 6) != round(t_h, 6):
                continue
            
            t = t_r
            total_height = float(row_r["height"]) + float(row_h["height"])
            total_height /= 1000

            times_2.append(t)
            total_height_list.append(total_height)

        average_height = find_room_height(roof_rows, height_rows)
        pos_z_h, pos_z_r, pos_z_avg = z_position_calc(roof_rows, height_rows, average_height)

        # Write results
        for t, vx, vy, px, py, pz, ax, ay in zip(times, vx_list, vy_list, pos_x, pos_y, pos_z_avg, acc_x, acc_y):
            writer.writerow([f"{t:.6f}", f"{vx:.6f}", f"{vy:.6f}", f"{px:.6f}", f"{py:.6f}", f"{pz:.6f}", f"{ax:.6f}", f"{ay:.6f}"])
        
        print(f"Wrote {len(times)} rows to {output_path}")    

    # === Plot velocities (single axes) ===
    plt.figure(figsize=(12, 6))
    plt.plot(times, vx_list, label="v_x (m/s)")
    plt.plot(times, vy_list, label="v_y (m/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Optical Flow Derived Velocities")
    plt.legend()
    
    plt.grid(True)
    plt.tight_layout()

if __name__ == "__main__":
    main()