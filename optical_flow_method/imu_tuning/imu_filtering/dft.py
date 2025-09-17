import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Load IMU CSV
file_path = "../../../optical_flow_method_data/combined_samples/13_09_25_MILC/straight1/download_imu_cropped.csv"
df = pd.read_csv(file_path)

# Extract accelerometer data
x_data = df["accel x"].to_numpy()
y_data = df["accel y"].to_numpy()
z_data = df["accel z"].to_numpy()

# Compute sampling rate from time column
timestamps = df["time"].to_numpy()
T_s = np.mean(np.diff(timestamps))
fs = 1.0 / T_s

# FFT
x_fft = np.fft.fft(x_data)
y_fft = np.fft.fft(y_data)
z_fft = np.fft.fft(z_data)
freq = np.fft.fftfreq(len(x_data), d=T_s)

# Plot FFT
plt.figure(figsize=(8, 18))

plt.subplot(3, 1, 1)
plt.plot(freq, np.abs(x_fft), color='blue')
plt.axvline(x=20, color='red', linestyle='--')
plt.axvline(x=-20, color='red', linestyle='--')
plt.title('Frequency Domain Data (DFT) - Accelerometer')
plt.ylabel('X Amplitude')

plt.subplot(3, 1, 2)
plt.plot(freq, np.abs(y_fft), color='orange')
plt.axvline(x=20, color='red', linestyle='--')
plt.axvline(x=-20, color='red', linestyle='--')
plt.ylabel('Y Amplitude')

plt.subplot(3, 1, 3)
plt.plot(freq, np.abs(z_fft), color='green')
plt.axvline(x=20, color='red', linestyle='--')
plt.axvline(x=-20, color='red', linestyle='--')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Z Amplitude')

plt.tight_layout()
plt.show()
