import pandas as pd

# Input and output file paths
input_file = "../optical_flow_method_data/imc45686_data_stationary.csv"
output_file = "../optical_flow_method_data/imc45686_data_stationary_scaled.csv"

# Read the CSV
df = pd.read_csv(input_file)

# Divide gyro columns by 100
df["gyro x"] = df["gyro x"] / 100.0
df["gyro y"] = df["gyro y"] / 100.0
df["gyro z"] = df["gyro z"] / 100.0

# Save to new CSV
df.to_csv(output_file, index=False)

print(f"Scaled file saved as {output_file}")
