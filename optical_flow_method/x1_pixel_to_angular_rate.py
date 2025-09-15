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
    
    # rad per pixel

    # s = math.radians(fov_deg) / res # original code

    # https://www.andrewgordon.me/posts/Adventures-in-Optical-Flow/
    # distance = (sensor_reading x distance_to_image / sensor_resolution ) x 2 x tan(field_of_view / 2.0)
    # distance = (sensor_reading x h / sensor_resolution ) x 2 x tan(field_of_view / 2.0)
    # s = 2 x tan(field_of_view / 2.0) / sensor_resolution
    s = 2*math.tan(fov_deg/2 * math.pi/180)/res # new code: unsure which is better yet


    # Note: max rate of 7.4 radians/second
    
    wx = (dx * s) / dt # this is multiplied by h to get v
    wy = (dy * s) / dt
    
    return wx, wy

def main(input_path, start_time, end_time):
    output_path = "../optical_flow_method_data/optical_flow_angular_rates.csv"
    
    with open(input_path, "r") as f_in, open(output_path, "w", newline="") as f_out:
        reader = csv.DictReader(f_in)
        writer = csv.writer(f_out)
        writer.writerow(["time (s)", "wx (rad/s)", "wy (rad/s)"])
        
        print(f"{'time':>12}  {'dx_sum':>7}  {'dy_sum':>7}  {'wx(rad/s)':>12}  {'wy(rad/s)':>12}")
        print("-" * 60)

        # # Buffers for 10-sample batching
        # batch_times = []
        # batch_dx = []
        # batch_dy = []

        for row in reader:
            t = float(row["time"])
            dx = int(row["deltaX"])
            dy = int(row["deltaY"])

            # why is t being input at dt?
            wx, wy = pixels_to_angular_rates(dx, dy, 0.1) # period of 0.1 for 10 Hz

            if t > start_time and t < end_time:
                print(f"{t:12.6f}  {dx:7d}  {dy:7d}  {wx:12.6f}  {wy:12.6f}")
                # Currently writing all samples
                writer.writerow([f"{t:.6f}", f"{wx:.6f}", f"{wy:.6f}"])


            # BELOW CODE WAS FOR 100 HZ to 10 HZ
            # batch_times.append(t)
            # batch_dx.append(dx)
            # batch_dy.append(dy)
            
            # last_time = 12.3
            # # Process batch every 10 samples
            # if len(batch_times) == 10:
            #     dt = batch_times[-1] - last_time
            #     dx_total = sum(batch_dx)
            #     dy_total = sum(batch_dy)

            #     wx, wy = pixels_to_angular_rates(dx_total, dy_total, dt) # keep this

            #     # Use the last time in the batch as the timestamp
            #     print(f"{batch_times[-1]:12.6f}  {dx_total:7d}  {dy_total:7d}  {wx:12.6f}  {wy:12.6f}")
            #     # Currently writing the first time of the sample of ten - open to change
            #     writer.writerow([f"{batch_times[-1]:.6f}", f"{wx:.6f}", f"{wy:.6f}"]) # keep this

            #     # Reset for next batch
            #     last_time = batch_times[-1]
            #     batch_times.clear()
            #     batch_dx.clear()
            #     batch_dy.clear()

if __name__ == "__main__":
    main()
