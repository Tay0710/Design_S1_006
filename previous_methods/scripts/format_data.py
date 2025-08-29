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
    
    # print("Timestamp:", timestamp)
    # print("Position (x, y, z): ", x, y, z)
    # print("Yaw:", yaw)
    # print("Distances:", distances)
    
    return timestamp, (x, y, z), yaw, distances   
    
    
# Read in the test data file
def read_file(filepath):
    # Create arrays for sets of data
    timestamps = []
    positions = []
    yaws = []
    distance_sets = []
    
    with open(filepath, 'r') as file:
        for line in file:
            timestamp, position, yaw, distances = read_line(line)
            
            timestamps.append(timestamp)
            positions.append(position)
            yaws.append(yaw)
            distance_sets.append(distances)
            
    return timestamps, positions, yaws, distance_sets
            
if __name__ == '__main__':
    read_file()