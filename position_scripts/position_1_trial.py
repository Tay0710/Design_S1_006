import pandas as pd
import numpy as np

def load_sensor_data(filepath):
    full_data = pd.read_csv(filepath)
    return full_data

def calculate_sample_rate(time):
    time_step = np.mean(np.diff(time))
    sample_rate = 1.0 / time_step
    return sample_rate, time_step
    
def main():
    df = load_sensor_data("../sensor_logs/sensor_data.csv")
    
    time = df["Time (s)"].values
    sample_rate, time_step = calculate_sample_rate(time)
    
    print(f"Sample rate: {sample_rate:.2f} Hz")
    print(f"Time step: {time_step:.6f} s")
    
if __name__ == "__main__":
    main()
