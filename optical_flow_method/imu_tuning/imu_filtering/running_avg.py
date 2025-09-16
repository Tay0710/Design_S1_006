import matplotlib.pyplot as plt
import csv

# Function to calculate running average
def running_average(data, window_size):
    return [sum(data[i:i+window_size]) / window_size for i in range(len(data) - window_size + 1)]

# Initialize empty lists to store accel data
x_data, y_data, z_data = [], [], []

# Read data from the IMU CSV file
file_path = "../../../optical_flow_method_data/combined_samples/rectangle/IMU_combined_rectangle.csv"

with open(file_path, 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip header row
    for row in reader:
        # accel x, accel y, accel z are columns 4, 5, 6
        x_data.append(float(row[4]))
        y_data.append(float(row[5]))
        z_data.append(float(row[6]))

# Define the window size for the running average
window_size = 250

# Calculate running averages for X, Y, and Z data
x_out = running_average(x_data, window_size)
y_out = running_average(y_data, window_size)
z_out = running_average(z_data, window_size)

# Adjust indices for plotting filtered output
idx = range(window_size - 1, len(x_data))

# Plotting
plt.figure(figsize=(8, 6))

# Raw vs filtered X
plt.plot(x_data, color='lightblue', linewidth=0.8, label='Raw Accel X')
plt.plot(idx, x_out, color='blue', linewidth=2, label='Filtered Accel X')

# Raw vs filtered Y
plt.plot(y_data, color='navajowhite', linewidth=0.8, label='Raw Accel Y')
plt.plot(idx, y_out, color='orange', linewidth=2, label='Filtered Accel Y')

# Raw vs filtered Z
plt.plot(z_data, color='lightgreen', linewidth=0.8, label='Raw Accel Z')
plt.plot(idx, z_out, color='green', linewidth=2, label='Filtered Accel Z')

plt.legend()
plt.xlabel('Sample')
plt.ylabel('Acceleration (g)')
plt.title('Accelerometer Data: Raw vs Running Average Filtered')
plt.tight_layout()
plt.show()