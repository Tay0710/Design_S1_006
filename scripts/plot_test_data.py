import matplotlib.pyplot as plt

# Clean the test data line by line
def read_line(line):
    #Remove unnecessary brackets and white spaces
    clean = line.strip()[1:-1]
    # print("Data: ", clean)
    
    #Split using delimiter ','
    values = clean.split(',')
    
    timestamp = values[0]
    distances = list(map(int, values[1:6]))
    yaw = float(values[7])
    
    print("Timestamp:", timestamp)
    print("Distances:", distances)  # Example: front sensor
    print("Yaw values:", yaw)
    
    return timestamp, distances, yaw   
    
    
# Read in the test data file
def read_file():
    with open('../sensor_logs/test_data_10_lines.txt', 'r') as file:
        for line in file:
            read_line(line)
            
if __name__ == '__main__':
    read_file()