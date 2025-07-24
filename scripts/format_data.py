# Clean the test data line by line
def read_line(line):
    #Remove unnecessary brackets and white spaces
    clean = line.strip()[1:-1]
    # print("Data: ", clean)
    
    #Split using delimiter ','
    values = clean.split(',')
    
    timestamp = values[0]
    x, y, z, = map(float, values[1:4])
    yaw = float(values[4])
    distances = list(map(int, values[5:11]))
    
    print("Timestamp:", timestamp)
    print("Position (x, y, z): ", x, y, z)
    print("Yaw:", yaw)
    print("Distances:", distances)
    
    return timestamp, (x, y, z), distances, yaw   
    
    
# Read in the test data file
def read_file():
    # Create arrays for sets of data
    timestamps = []
    positions = []
    yaws = []
    distance_sets = []
    
    
    with open('../sensor_logs/test_data_10_lines.txt', 'r') as file:
        for line in file:
            timestamp, position, yaw, distances = read_line(line)
            
            timestamps.append(timestamp)
            positions.append(position)
            yaws.append(yaw)
            distance_sets.append(distances)
            
    return timestamps, positions, yaws, distance_sets
            
if __name__ == '__main__':
    read_file()