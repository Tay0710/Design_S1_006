import pandas as pd

# Load the CSV file
df = pd.read_csv("../sensor_logs/sensor_data.csv")

# Preview first few rows
print("First 5 rows:")
print(df.head())

# Check columns and datatypes
print("\nColumn info:")
print(df.dtypes)

# Check if there are missing values
print("\nAny NaNs?")
print(df.isna().any())
