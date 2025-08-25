import math
import csv
import numpy as np
import ast
# Maybe an outlier fn - but how can you make sure this doesn't include the dodgy one
# 163.28125 is average - maybe twice this as outlier cancel


def calculate_height(arr):
    """
    The field of view (fov) of the VL53L7CX_2 is 90 degrees
    The angle between each data point is 90/7 = 12.857
    The angle from the central measurements to the middle of the fov is 12.857/2 = 6.429
    """
    h1 = arr[3,3] * math.cos(6.429)
    h2 = arr[3,4] * math.cos(6.429)
    h3 = arr[4,3] * math.cos(6.429)
    h4 = arr[4,4] * math.cos(6.429)    
    
    print(f"[calculate_height] arr1 = {arr[3,3]}")
    print(f"[calculate_height] arr2 = {arr[3,4]}")
    print(f"[calculate_height] arr3 = {arr[4,3]}")
    print(f"[calculate_height] arr4 = {arr[4,4]}")
          
    h_ave = (h1 + h2 + h3 + h4) / 4
    h_rounded = round(h_ave, 3)
    print(f"[calculate_height] h_ave = ({h1} + {h2} + {h3} + {h4}) / 4 = {h_rounded}")
    
    return h_rounded

def read_file(fp):
    times = []
    arrays = []
    
    with open(fp, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row["time (s)"])
            times.append(t)
            
            arr = np.array(ast.literal_eval(row["array"]), dtype=float)
            
            if arr.shape != (8, 8):
                raise ValueError(f"Row at time {t} did not contain an 8x8 array")
            
            arrays.append(arr)
            
    return np.array(times), np.array(arrays)

def main():
    input_path = "../optical_flow_method_data/ToF_test_log.csv"
    output_path = "../optical_flow_method_data/ToF_heights.csv"
    
    times, arrays = read_file(input_path)
    print("Times shape:", times.shape)       # (N,)
    print("Arrays shape:", arrays.shape)     # (N, 8, 8)
    
    heights = []
    for i, (t, frame) in enumerate(zip(times, arrays)):
        print(f"\n[frame {i}] t={t}")
        h = calculate_height(frame)  # <-- pass a single (8,8) frame
        heights.append(h)

if __name__ == "__main__":
    main()