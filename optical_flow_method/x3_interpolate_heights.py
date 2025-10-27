"""
x3_interpolate_heights.py
-------------------------
Interpolates ToF-derived height data to align with optical-flow timestamps.

Purpose:
    Resample ToF height measurements onto optical-flow sample times for use in
    velocity and fusion stages.

Overview:
    Optical flow and ToF run at different rates. This stage loads ToF heights and
    optical-flow timestamps, performs linear interpolation, and writes synchronized
    (time, height) pairs. Optional plotting compares original vs interpolated data.

Usage:
    Called from x0_position_pipeline.py during Stage 3.

Inputs:
    ../optical_flow_method_data/ToF_heights.csv
        Columns: time (s), height (mm)
    ../optical_flow_method_data/ToF_roof.csv
        Columns: time (s), height (mm)
    ../optical_flow_method_data/optical_flow_angular_rates.csv
        Columns: time (s), wx (rad/s), wy (rad/s)

Outputs:
    ../optical_flow_method_data/ToF_heights_interp.csv
        Columns: time (s), height (mm)
    ../optical_flow_method_data/ToF_roof_interp.csv
        Columns: time (s), height (mm)
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def load_heights(height_path):
    """Load ToF heights CSV --> np arrays (time, height)."""
    
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
    """Load OF timestamps (time (s)) --> np array."""
    
    times = []
    
    with open(of_path, "r") as f_ang:
        reader = csv.DictReader(f_ang)
        for row in reader:
            times.append(float(row["time (s)"]))
    return np.array(times)

def interpolate_heights(height_times, heights, of_times, output_path):
    """Linear interpolation of heights onto OF times; writes CSV."""
    
    f_interp = interp1d(height_times, heights, kind="linear", fill_value="extrapolate")
    h_interp = f_interp(of_times)

    with open(output_path, "w", newline="") as f_out:
        writer = csv.writer(f_out)
        writer.writerow(["time", "height"])
        for t, h in zip(of_times, h_interp):
            writer.writerow([f"{t:.6f}", f"{h:.6f}"])

    print(f"[interpolate_heights] wrote {len(of_times)} rows to {output_path}")

    return of_times, h_interp

def plot_heights(original_times_h, original_times_r, original_heights, original_roof, interp_times_h, interp_times_r, interp_heights, interp_roof):
    """Plot original vs interpolated heights for floor and roof."""
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

    # Top plot
    ax1.plot(original_times_h, original_heights, "o-", label="Original ToF samples", markersize=6)
    ax1.plot(interp_times_h, interp_heights, "x", label="Interpolated at OF times", markersize=6, color="red")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Height (mm)")
    ax1.set_title("Height Interpolation at Optical Flow Timestamps - Floor")
    ax1.legend()
    ax1.grid(True)

    # Bottom plot
    ax2.plot(original_times_r, original_roof, "o-", label="Original ToF samples", markersize=6)
    ax2.plot(interp_times_r, interp_roof, "x", label="Interpolated at OF times", markersize=6, color="red")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Height (mm)")
    ax2.set_title("Height Interpolation at Optical Flow Timestamps - Roof")
    ax2.legend()
    ax2.grid(True)

    # Adjust spacing
    plt.tight_layout()

    # Show the plot
    plt.show()

def main():
    """Interpolate floor/roof heights to OF timestamps and plot."""
    
    height_path = "../optical_flow_method_data/ToF_heights.csv"
    angular_rate_path = "../optical_flow_method_data/optical_flow_angular_rates.csv"
    output_path_h = "../optical_flow_method_data/ToF_heights_interp.csv"

    roof_path = "../optical_flow_method_data/ToF_roof.csv"
    output_path_r = "../optical_flow_method_data/ToF_roof_interp.csv"

    # Interpolate distance from floor
    t_h, h_vals = load_heights(height_path)
    t_of = load_of_times(angular_rate_path)
    of_times_1, h_interp = interpolate_heights(t_h, h_vals, t_of, output_path_h)

    # Interpolate distance from roof
    t_r, r_vals = load_heights(roof_path)
    of_times_2, r_interp = interpolate_heights(t_r, r_vals, t_of, output_path_r)

    # Plot results
    plot_heights(t_h, t_r, h_vals, r_vals, of_times_1, of_times_2, h_interp, r_interp)

if __name__ == "__main__":
    main()