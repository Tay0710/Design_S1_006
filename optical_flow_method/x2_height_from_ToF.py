"""
x2_height_from_ToF.py
---------------------
Processes raw VL53L7CX ToF sensor data to estimate drone height.

Overview:
    The VL53L7CX is an 8×8 ToF sensor with a 90° field of view (FOV).
    We select 4 central measurement zones (D6, D5, D10, D9) around the 
    center of the array, adjust each measurement for the sensor angle, 
    and average them to obtain an estimate of drone altitude.

    This assumes the sensor is mounted perpendicular to the floor.

Notes:
    - FOV = 90°
    - Pixel pitch = 90 / 7 = 12.857°
    - Offset angle from central measurement to FOV mid = 6.429°
    - Correction: h = d × cos(θ)

Inputs:
    ToF_combined_square2.csv
        Columns: 
            time, D0 … D63 (distance in mm per zone)

Outputs:
    ToF_heights.csv
        Columns:
            time (s), height (mm)
        where height is the averaged, corrected altitude.

"""

import math
import csv
import numpy as np
import ast
# Maybe an outlier fn - but how can you make sure this doesn't include the dodgy one
# 163.5125 is average - maybe twice this as outlier cancel

def calculate_height(D5, D6, D9, D10, last_height):
    """
    These are altered to suit a 4x4 array as this is currently the most accurate
    The field of view (fov) of the VL53L7CX_2 is 90 degrees
    The angle between each data point is 90/3 = 30
    The angle from the central measurements to the middle of the fov is 30/2 = 15
    """
    
    angle = 15 * math.pi/180
    valid = []
    
    # Check each input, skip if it's "x" or nan
    for val in [D5, D6, D9, D10]:
        if isinstance(val, str):
            if val.lower() == "x":
                continue
            try:
                v = float(val)
            except ValueError:
                continue
        else:
            v = float(val)

        if np.isnan(v):   # skip NaN values
            continue
        valid.append(v * math.cos(angle))
            
    if len(valid) == 0:
        print("[calculate_height] All values invalid, using last_height =", last_height)
        return last_height
    
    print(f"[calculate_height] arr1 = {D5}")
    print(f"[calculate_height] arr2 = {D6}")
    print(f"[calculate_height] arr3 = {D9}")
    print(f"[calculate_height] arr4 = {D10}")
          
    h_ave = sum(valid) / len(valid)
    h_rounded = round(h_ave, 3)
    print(f"[calculate_height] h_ave = {h_rounded} from {len(valid)} valid values")

    return h_rounded

def main(input_path):
    output_path = "../optical_flow_method_data/ToF_heights.csv"

    last_height = 0.0  # initial fallback value

    with open(input_path, "r") as f_in, open(output_path, "w", newline="") as f_out:
        reader = csv.DictReader(f_in)  # handles "time,type,D0...D63"
        writer = csv.writer(f_out)
        writer.writerow(["time", "height"])

        for row in reader:
            if row["type"] != "D":   # only process type D rows
                continue

            t = float(row["time"])
            h = calculate_height(row["D5"], row["D6"], row["D9"], row["D10"], last_height)
            last_height = h
            writer.writerow([f"{t:.6f}", f"{h:.6f}"])

            
if __name__ == "__main__":
    main()