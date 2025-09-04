import math
import csv

# Convert PMW3901 pixel deltas to angular-rates (rad/s)
def pixels_to_angular_rates(dx, dy, dt):
    res = 35.0
    fov_deg = 42.0
    if dt is None:
        return 0.0, 0.0
    
    # rad per pixel
    s = math.radians(fov_deg)/ res
    
    wx = (dx * s) / dt
    wy = (dy * s) / dt
    
    return wx, wy

# Calculate dt and variable validation
def compute_dt(t_curr, t_prev):
    if t_prev or t_curr is None:
        return None
    if t_curr > t_prev:
        dt = t_curr - t_prev
        return dt
    else:
        return None

def main():
    input_path = "../optical_flow_method_data/combined_samples/square2/OF_combined_square2.csv"
    output_path = "../optical_flow_method_data/optical_flow_angular_rates.csv"
    
    with open(input_path, "r") as f_in, open(output_path, "w", newline="") as f_out:
        reader = csv.DictReader(f_in)
        writer = csv.writer(f_out)
        writer.writerow(["time (s)", "wx (rad/s)", "wy (rad/s)"])
        
        t_prev = 0.0
        
        print(f"{'time':>6}  {'dx':>4}  {'dy':>4}  {'wx(rad/s)':>12}  {'wy(rad/s)':>12}")
        print("-" * 50)
        
        for row in reader:
            t = float(row["time"])
            dx = int(row["deltaX"])
            dy = int(row["deltaY"])
            
            dt = compute_dt(t, t_prev)
            
            wx, wy = pixels_to_angular_rates(dx, dy, dt)
            print(f"{t:6.2f}  {dx:4d}  {dy:4d}  {wx:12.6f}  {wy:12.6f}")
            
            # Write to CSV
            writer.writerow([f"{t:.6f}", f"{wx:.6f}", f"{wy:.6f}"])
            
            # ✅ Update previous time
            t_prev = t
            
if __name__ == "__main__":
    main()