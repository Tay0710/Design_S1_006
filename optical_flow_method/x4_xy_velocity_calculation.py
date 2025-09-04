import csv

def velocity_calc(wx, wy, h):
    v_x = wx * h
    v_y = wy * h
    return v_x, v_y

def main():
    angular_rate_path = "../optical_flow_method_data/optical_flow_angular_rates.csv"
    height_path = "../optical_flow_method_data/ToF_heights_interp.csv"
    output_path = "../optical_flow_method_data/xy_velocities.csv"
    
    with open(angular_rate_path, "r") as f_angular, \
        open(height_path, "r") as f_height, \
        open(output_path, "w", newline="") as f_out:
        
        reader_angular = csv.DictReader(f_angular)
        reader_height = csv.DictReader(f_height)
        writer = csv.writer(f_out)
        writer.writerow(["time (s)", "v_x (mm/s)", "v_y (mm/s)"])
        
        for i, (row_a, row_h) in enumerate(zip(reader_angular, reader_height), start=1):
            
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
            writer.writerow([f"{t:.6f}", f"{v_x:.6f}", f"{v_y:.6f}"])
        
        print(f"Wrote {i} rows to {output_path}")            

if __name__ == "__main__":
    main()