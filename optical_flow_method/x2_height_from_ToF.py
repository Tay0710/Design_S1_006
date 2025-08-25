import csv
import numpy as np
import ast
# Maybe an outlier fn - but how can you make sure this doesn't include the dodgy one
# 163.28125 is average - maybe twice this as outlier cancel


def calculate_height(height_array):
    
    return 0

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

if __name__ == "__main__":
    main()