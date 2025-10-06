import numpy as np

# data = np.genfromtxt("../optical_flow_method_data/sensor_data.csv", delimiter=",", skip_header=1)
data = np.genfromtxt("../optical_flow_method_data/imc45686_data_square.csv", delimiter=",", skip_header=1)

timestamp = data[:, 0]
gyroscope = data[:, 1:4]       # [deg/s]
accelerometer = data[:, 4:7]   # [g]

sample_time_s = np.mean(np.diff(timestamp))/1000000
sample_rate = int(round(1.0 / sample_time_s))
print("Sample Rate: ", sample_rate)

