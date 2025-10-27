"""
x1_pixel_to_angular_rate.py
---------------------------
Stage 1 of the ELEC5550 Indoor 3D Mapping Design Project (2025) position pipeline.

Purpose:
    Convert PMW3901 pixel deltas (dx, dy) into angular velocities (ωx, ωy) in rad/s.

Overview:
    Applies a pixels --> radians scale factor s and divides by sample Δt:
        ωx = (dx × s) / Δt,  ωy = (dy × s) / Δt.
    Default s uses the project’s empirical constant.

Usage:
    Called from x0_position_pipeline.py.

Inputs:
    Optical flow data CSV
        Columns: time, deltaX, deltaY.

Outputs:
    ../optical_flow_method_data/optical_flow_angular_rates.csv
        Columns: time (s), wx (rad/s), wy (rad/s).
"""

import math
import csv

def pixels_to_angular_rates(dx, dy, dt):
    '''
    Convert pixel deltas to angular rates (rad/s).
    
    PMW3901 angular scale s (rad/pixel)
    Approximate PMW3901 angular-rate limit ≈ 7.4 rad/s.

    Options to decide scale factor (choose one and set s):

    OPTION 1 — FOV linear:
        s = radians(fov_deg) / res

    OPTION 2 — Tangent mapping:
        s = 2 * tan(radians(fov_deg) / 2) / res
        References: https://www.andrewgordon.me/posts/Adventures-in-Optical-Flow/
                    https://kamathsblog.com/odometry-using-optical-flow

    OPTION 3 — Project constants (MILC floor):
        s = 0.0015
        or
        s = 0.09 / 57.3
        References: https://isif.org/files/isif/2025-01/optical_flow_p72.pdf

    OPTION 4 — Empirical fit:
        s = 0.0015706806282723
        or
        s = 0.00188

    General References:
                    https://circuitdigest.com/microcontroller-projects/interfacing-pmw3901-optical-flow-sensor-with-esp32
                    https://forum.bitcraze.io/viewtopic.php?t=2882&start=10
    '''

    res = 35.0
    fov_deg = 42.0
    if dt is None or dt <= 0:
        return 0.0, 0.0
    
    s = 0.00188

    wx = (dx * s) / dt
    wy = (dy * s) / dt
    
    return wx, wy

def main(input_path):
    """Read optical flow CSV and write angular rates CSV."""
    
    output_path = "../optical_flow_method_data/optical_flow_angular_rates.csv"
    
    with open(input_path, "r") as f_in, open(output_path, "w", newline="") as f_out:
        reader = csv.DictReader(f_in)
        writer = csv.writer(f_out)
        writer.writerow(["time (s)", "wx (rad/s)", "wy (rad/s)"])
        
        print(f"{'time':>12}  {'dx_sum':>7}  {'dy_sum':>7}  {'wx(rad/s)':>12}  {'wy(rad/s)':>12}")
        print("-" * 60)

        last_time = 0

        for row in reader:
            t = float(row["time"])
            dx = int(row["deltaX"])
            dy = int(row["deltaY"])

            dt = t - last_time
            last_time = t
            
            wx, wy = pixels_to_angular_rates(dx, dy, dt)
            
            print(f"{t:12.6f} {dt:12.6f} {dx:7d}  {dy:7d}  {wx:12.6f}  {wy:12.6f}")
            writer.writerow([f"{t:.6f}", f"{wx:.6f}", f"{wy:.6f}"])

if __name__ == "__main__":
    main()