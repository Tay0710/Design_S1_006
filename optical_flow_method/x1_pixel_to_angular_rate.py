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
    # https://circuitdigest.com/microcontroller-projects/interfacing-pmw3901-optical-flow-sensor-with-esp32
    res = 35.0
    fov_deg = 42.0
    if dt is None or dt <= 0:
        return 0.0, 0.0
    
    # check this: https://forum.bitcraze.io/viewtopic.php?t=2882&start=10 
    
    # rad per pixel

    # OPTION 1:
    # s = math.radians(fov_deg) / res 

    # OPTION 2:
    # https://www.andrewgordon.me/posts/Adventures-in-Optical-Flow/
    # distance = (sensor_reading x distance_to_image / sensor_resolution ) x 2 x tan(field_of_view / 2.0)
    # distance = (sensor_reading x h / sensor_resolution ) x 2 x tan(field_of_view / 2.0)
    # s = 2 x tan(field_of_view / 2.0) / sensor_resolution
    # https://kamathsblog.com/odometry-using-optical-flow
    # s = 2*math.tan(fov_deg/2 * math.pi/180)/(res) # new code: unsure which is better yet

    # OPTION 3:
    # This works specifically for MILC Floor measurements
    # https://isif.org/files/isif/2025-01/optical_flow_p72.pdf
    # s = 0.0015
    s = 0.09/57.3 # from matlab sim


    # Note: max rate of 7.4 radians/second
    
    wx = (dx * s) / dt # this is multiplied by h to get v
    wy = (dy * s) / dt
    
    return wx, wy

def main(input_path):
    output_path = "../optical_flow_method_data/optical_flow_angular_rates.csv"
    
    with open(input_path, "r") as f_in, open(output_path, "w", newline="") as f_out:
        reader = csv.DictReader(f_in)
        writer = csv.writer(f_out)
        writer.writerow(["time (s)", "wx (rad/s)", "wy (rad/s)"])
        
        print(f"{'time':>12}  {'dx_sum':>7}  {'dy_sum':>7}  {'wx(rad/s)':>12}  {'wy(rad/s)':>12}")
        print("-" * 60)


        for row in reader:
            t = float(row["time"])
            dx = int(row["deltaX"])
            dy = int(row["deltaY"])

            
            wx, wy = pixels_to_angular_rates(dx, dy, 0.01) # period of 0.1 for 10 Hz

            
            print(f"{t:12.6f}  {dx:7d}  {dy:7d}  {wx:12.6f}  {wy:12.6f}")
            # Currently writing all samples
            writer.writerow([f"{t:.6f}", f"{wx:.6f}", f"{wy:.6f}"])



if __name__ == "__main__":
    main()
