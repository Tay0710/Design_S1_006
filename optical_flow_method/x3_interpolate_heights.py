"""
x3_interpolate_heights.py
-------------------------
Interpolates ToF-derived height data to align with optical flow timestamps.

Overview:
    Optical flow and ToF sensors operate at different sample rates.
    To combine them for velocity estimation, height measurements from 
    the ToF sensor must be resampled to match the optical flow timestamps.

    This script:
        1. Loads ToF height measurements (time, height).
        2. Loads optical flow angular-rate timestamps.
        3. Performs linear interpolation to estimate height values at 
           each optical flow timestamp.
        4. Outputs synchronized (time, height) pairs to CSV.
        5. Optionally visualizes the interpolation.

Inputs:
    - ToF_heights.csv
        Columns:
            time (s), height (mm)
    - optical_flow_angular_rates.csv
        Columns:
            time (s), wx (rad/s), wy (rad/s)

Outputs:
    - ToF_heights_interp.csv
        Columns:
            time (s), height (mm)
        Interpolated heights aligned to optical flow sample times.
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def load_heights(height_path):
    times, heights = [], []
    with open(height_path, "r") as f_height:
        reader = csv.DictReader(f_height)
        for row in reader:
            t = float(row["time"])
            h = float(row["height"])
            times.append(t)
            heights.append(h)
    return np.array(times), np.array(heights)

def load_of_times(of_path):
    times = []
    with open(of_path, "r") as f_ang:
        reader = csv.DictReader(f_ang)
        for row in reader:
            times.append(float(row["time (s)"]))
    return np.array(times)

def interpolate_heights(height_times, heights, of_times, output_path):
    f_interp = interp1d(height_times, heights, kind="linear", fill_value="extrapolate")
    h_interp = f_interp(of_times)

    with open(output_path, "w", newline="") as f_out:
        writer = csv.writer(f_out)
        writer.writerow(["time", "height"])
        for t, h in zip(of_times, h_interp):
            writer.writerow([f"{t:.6f}", f"{h:.6f}"])

    print(f"[interpolate_heights] wrote {len(of_times)} rows to {output_path}")

    return of_times, h_interp

def plot_heights(original_times, original_heights, interp_times, interp_heights):
    plt.figure(figsize=(10, 5))
    plt.plot(original_times, original_heights, "o-", label="Original ToF samples", markersize=6)
    plt.plot(interp_times, interp_heights, "x", label="Interpolated at OF times", markersize=6, color="red")
    plt.xlabel("Time (s)")
    plt.ylabel("Height (mm)")
    plt.title("Height Interpolation at Optical Flow Timestamps")
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    height_path = "../optical_flow_method_data/ToF_heights.csv"
    angular_rate_path = "../optical_flow_method_data/optical_flow_angular_rates.csv"
    output_path = "../optical_flow_method_data/ToF_heights_interp.csv"

    t_h, h_vals = load_heights(height_path)
    t_of = load_of_times(angular_rate_path)
    of_times, h_interp = interpolate_heights(t_h, h_vals, t_of, output_path)

    # Plot results
    plot_heights(t_h, h_vals, of_times, h_interp)

if __name__ == "__main__":
    main()
