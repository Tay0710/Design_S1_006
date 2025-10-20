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
    pos_x = np.zeros(len(times-1))
    pos_y = np.zeros(len(times-1))

    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        pos_x[i] = pos_x[i-1] + 0.5 * dt * (vx_list[i] + vx_list[i-1])
        pos_y[i] = pos_y[i-1] + 0.5 * dt * (vy_list[i] + vy_list[i-1])

    return pos_x, pos_y

def differentiate_velocities(times, vx_list, vy_list):
    """Differentiate velocities into acceleration using numerical methods."""
    acc_x = np.zeros(len(times-1))
    acc_y = np.zeros(len(times-1))

    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        acc_x[i] = (vx_list[i]-vx_list[i-1])/dt
        acc_y[i] = (vy_list[i]-vy_list[i-1])/dt

    return acc_x, acc_y

def find_room_height(roof_rows, height_rows):
    sum = 0
    length = 0
    start = 0
    averaging_period = 5 # in seconds

    for row_r, row_h in zip(roof_rows, height_rows):
            # read in both timestamps
            t_r = float(row_r["time"])
            t_h = float(row_h["time"])

            # only proceed if timestamps match (allow noise for now)
            if round(t_r, 6) != round(t_h, 6):
                continue

            if length == 0:
                start = t_r
            
            # Break out of for loop after the averaging period ends
            if round(t_r, 6) > start + averaging_period:
                break
            
            t = t_r
            total_height = float(row_r["height"]) + float(row_h["height"])
            total_height /= 1000 # convert to m from mm

            sum += total_height
            length += 1

    average_height = sum/length
    return average_height

def z_position_calc(roof_rows, height_rows, average_height):
    pos_z_h = []
    pos_z_r = []
    pos_z_avg = []
    altitude_thresh = 0.05 # 5cm

    # TODO: what happens if there is an obstacle above and below

    # 1. if total height is significantly less than average height (add a threshold)??
    # but this wont really happen due to extrapolation... the only way we can fix this is by saving autonomous commands

    # Assume no obstacles on the floor or roof for now

    for row_r, row_h in zip(roof_rows, height_rows):
        h_r = average_height - float(row_r["height"])/1000
        pos_z_r.append(h_r)

        h_h = float(row_h["height"])/1000
        pos_z_h.append(h_h)

        total_height = float(row_r["height"]) + float(row_h["height"])
        if (total_height/1000 - average_height < -altitude_thresh):
            h_avg = h_r
        else:
            h_avg = (h_h + h_r)/2
        pos_z_avg.append(h_avg)

    return pos_z_h, pos_z_r, pos_z_avg


def main():
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
        
        # First read all values into lists
        angular_rows = list(reader_angular)
        height_rows = list(reader_height)
        roof_rows = list(reader_roof)
        
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

            v_y = -v_y # negate y_velocity as OF axis y-axis is opposite to IMU y_axis

            times.append(t)
            vx_list.append(v_x)
            vy_list.append(v_y)

        # Integrate to positions
        pos_x, pos_y = integrate_positions(np.array(times), np.array(vx_list), np.array(vy_list))

        # Differentiate to acceleration
        acc_x, acc_y = differentiate_velocities(np.array(times), np.array(vx_list), np.array(vy_list))

        # Average the height of the room inthe first 5 seconds (assume drone is kept still in the first 5 seconds)
        for row_r, row_h in zip(roof_rows, height_rows):
            # read in both timestamps
            t_r = float(row_r["time"])
            t_h = float(row_h["time"])

            # only proceed if timestamps match (allow noise for now)
            if round(t_r, 6) != round(t_h, 6):
                continue
            
            t = t_r
            total_height = float(row_r["height"]) + float(row_h["height"])
            total_height /= 1000 # convert to m from mm

            times_2.append(t)
            total_height_list.append(total_height)

        average_height = find_room_height(roof_rows, height_rows)
        pos_z_h, pos_z_r, pos_z_avg = z_position_calc(roof_rows, height_rows, average_height)

        # Write results with positions
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


    # # === Plot trajectory (XY) ===
    # plt.subplot(2, 2)
    # plt.plot(pos_x, pos_y, "-o", markersize=2)
    # plt.xlabel("X position (m)")
    # plt.ylabel("Y position (m)")
    # plt.title("Integrated XY Position")
    # plt.axis("equal")
    # plt.grid(True)

    # # === Plot trajectory (XY) ===
    # plt.subplot(2, 2, 3)
    # plt.plot(times, acc_x, label="acc_x (m/s^2)", color="blue")
    # plt.plot(times, acc_y, label="acc_y (m/s^2)", color="red")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Acceleration (m/s^2)")
    # plt.title("Optical Flow Differentiated Acceleration")
    # plt.legend()
    # plt.grid(True)

    # # === Plot height of map (Z) === 
    # plt.subplot(2, 2, 4)
    # plt.plot(times_2, total_height_list, label="Total height", color="blue")
    # plt.axhline(y=average_height, color='blue', linestyle='--', label='Average height')
    # plt.plot(times_2, pos_z_h, label="Z position (m) - From Bottom ", color="red")
    # plt.plot(times_2, pos_z_r, label="Z position (m) - From Top" , color="green")
    # plt.plot(times_2, pos_z_avg, label="Z position (m) - Average" , color="black", linestyle='--')

    # plt.xlabel("Time (s)")
    # plt.ylabel("Total Height (m)")
    # plt.title("ToF Heights")
    # plt.legend()
    # plt.grid(True)


    # plt.tight_layout()
    # plt.show()

if __name__ == "__main__":
    main()