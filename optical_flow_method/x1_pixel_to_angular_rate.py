"""
x1_pixel_to_angular_rate.py
---------------------------
Converts PMW3901 optical flow pixel deltas into angular velocity estimates.

Overview:
    The PMW3901 sensor outputs pixel displacement (dx, dy) over time. This script
    batches 10 consecutive samples, sums their displacements, and converts them 
    into angular rates about the x and y axes (ωx, ωy) in radians/second.

Method:
    - Field of View (FOV): 42°
    - Sensor resolution (res): 35 pixels across FOV
    - Conversion factor: (FOV [rad] / resolution [pixels]) → rad/pixel
    - Angular velocity: (Δpixels × rad/pixel) / Δtime

Inputs:
    Optical flow data:  
        Columns:
            time, deltaX, deltaY

Outputs:
    optical_flow_angular_rates.csv
        Columns:
            time (s), wx (rad/s), wy (rad/s)
"""

import math
import csv

# Convert PMW3901 pixel deltas to angular-rates (rad/s)
def pixels_to_angular_rates(dx, dy, dt):
    res = 35.0*35
    fov_deg = 42.0
    if dt is None or dt <= 0:
        return 0.0, 0.0
    
    # rad per pixel
    s = math.radians(fov_deg) / res
    
    wx = (dx * s) / dt
    wy = (dy * s) / dt
    
    return wx, wy

def main():
    input_path = "../optical_flow_method_data/combined_samples/MILC_carpet/s1/download_of (1).csv"
    output_path = "../optical_flow_method_data/optical_flow_angular_rates.csv"
    
    with open(input_path, "r") as f_in, open(output_path, "w", newline="") as f_out:
        reader = csv.DictReader(f_in)
        writer = csv.writer(f_out)
        writer.writerow(["time (s)", "wx (rad/s)", "wy (rad/s)"])
        
        print(f"{'time':>12}  {'dx_sum':>7}  {'dy_sum':>7}  {'wx(rad/s)':>12}  {'wy(rad/s)':>12}")
        print("-" * 60)

        # Buffers for 10-sample batching
        batch_times = []
        batch_dx = []
        batch_dy = []


        for row in reader:
            t = float(row["time"])
            dx = int(row["deltaX"])
            dy = int(row["deltaY"])

            batch_times.append(t)
            batch_dx.append(dx)
            batch_dy.append(dy)
            
            last_time = 12.3
            # Process batch every 10 samples
            if len(batch_times) == 10:
                dt = batch_times[-1] - last_time
                dx_total = sum(batch_dx)
                dy_total = sum(batch_dy)

                wx, wy = pixels_to_angular_rates(dx_total, dy_total, dt)

                # Use the last time in the batch as the timestamp
                print(f"{batch_times[-1]:12.6f}  {dx_total:7d}  {dy_total:7d}  {wx:12.6f}  {wy:12.6f}")
                # Currently writing the first time of the sample of ten - open to change
                writer.writerow([f"{batch_times[-1]:.6f}", f"{wx:.6f}", f"{wy:.6f}"])

                # Reset for next batch
                last_time = batch_times[-1]
                batch_times.clear()
                batch_dx.clear()
                batch_dy.clear()

if __name__ == "__main__":
    main()
