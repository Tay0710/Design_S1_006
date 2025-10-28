"""
x2_height_from_ToF.py
---------------------
Stage 2 of the ELEC5550 Indoor 3D Mapping Design Project (2025) position pipeline.

Purpose:
    Estimate drone altitude from VL53L7CX distance measurements.

Overview:
    Selects four central zones (D6, D5, D10, D9) in the 4 x 4 grid, applies an off-axis correction
    using h = d × cos(15°), and averages the corrected values to obtain height.

    Notes: FOV = 90°, pixel pitch = 90° / 3 = 30°, offset from center = 30° / 2 = 15°,
    assumes the sensor is perpendicular to the floor.

Usage:
    Called from x0_position_pipeline.py during Stage 2.

Inputs:
    ToF CSV with per-zone distances (mm)
        Columns:
            time, D0 … D15, type

Outputs:
    ../optical_flow_method_data/ToF_heights.csv
        Columns:
            time (s), height (mm)
    ../optical_flow_method_data/ToF_roof.csv
        Columns:
            time (s), height (mm)
"""

import math
import csv
import numpy as np
import ast

def calculate_height(D5, D6, D9, D10, last_height):
    """Average 4 central zones (mm) with cos(15°) correction. If no average is available use last_height."""
    
    angle = 15 * math.pi / 180
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

        if np.isnan(v):
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
    """Compute floor and roof heights (mm) from VL53L7CX CSV."""
    
    output_path = "../optical_flow_method_data/ToF_heights.csv"

    last_height = 0.0

    with open(input_path, "r") as f_in, open(output_path, "w", newline="") as f_out:
        reader = csv.DictReader(f_in)
        writer = csv.writer(f_out)
        writer.writerow(["time", "height"])

        for row in reader:
            if row["type"] != "D":
                continue

            t = float(row["time"])
            h = calculate_height(row["D5"], row["D6"], row["D9"], row["D10"], last_height)
            last_height = h
            writer.writerow([f"{t:.6f}", f"{h:.6f}"])

    output_path = "../optical_flow_method_data/ToF_roof.csv"

    last_roof = 0

    with open(input_path, "r") as f_in, open(output_path, "w", newline="") as f_out:
        reader = csv.DictReader(f_in)
        writer = csv.writer(f_out)
        writer.writerow(["time", "height"])

        for row in reader:
            if row["type"] != "U":
                continue

            t = float(row["time"])
            h = calculate_height(row["D5"], row["D6"], row["D9"], row["D10"], last_roof)
            last_height = h
            writer.writerow([f"{t:.6f}", f"{h:.6f}"])
            
if __name__ == "__main__":
    main()