import math
import csv
import numpy as np
import ast
# Maybe an outlier fn - but how can you make sure this doesn't include the dodgy one
# 163.28125 is average - maybe twice this as outlier cancel

def calculate_height(D28, D27, D36, D35):
    """
    The field of view (fov) of the VL53L7CX_2 is 90 degrees
    The angle between each data point is 90/7 = 12.857
    The angle from the central measurements to the middle of the fov is 12.857/2 = 6.429
    """
    h1 = D28 * math.cos(6.429)
    h2 = D27 * math.cos(6.429)
    h3 = D36 * math.cos(6.429)
    h4 = D35 * math.cos(6.429)    
    
    print(f"[calculate_height] arr1 = {D28}")
    print(f"[calculate_height] arr2 = {D27}")
    print(f"[calculate_height] arr3 = {D36}")
    print(f"[calculate_height] arr4 = {D35}")
          
    h_ave = (h1 + h2 + h3 + h4) / 4
    h_rounded = round(h_ave, 3)
    print(f"[calculate_height] h_ave = ({h1} + {h2} + {h3} + {h4}) / 4 = {h_rounded}")
    
    return h_rounded

def main():
    input_path = "../optical_flow_method_data/combined_samples/square2/ToF_combined_square2.csv"
    output_path = "../optical_flow_method_data/ToF_heights.csv"

    # Load CSV (skip header)
    data = np.genfromtxt(input_path, delimiter=",", skip_header=1)

    # First column is time
    times = data[:, 0]

    # Grab only the requested columns
    d28 = data[:, 29]  # D28 is the 29th column (0-based index)
    d27 = data[:, 28]
    d36 = data[:, 37]
    d35 = data[:, 36]

    with open(output_path, "w", newline="") as f_out:
        writer = csv.writer(f_out)
        writer.writerow(["time", "height"])

        for t, v28, v27, v36, v35 in zip(times, d28, d27, d36, d35):
            h = calculate_height(v28, v27, v36, v35)
            writer.writerow([f"{t:.6f}", f"{h:.6f}"])
if __name__ == "__main__":
    main()