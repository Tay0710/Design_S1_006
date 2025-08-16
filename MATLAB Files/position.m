% Load CSV file in the same folder as the script
filename = '2025-08-12 18-07-55.csv';
data = readtable(filename);

% Extract columns
timestamp = data{:,1};  
gyroscopeReadings    = data(:,2:4);
accelerometerReadings= data(:,5:7);

% Calculate sample rate from timestamps
dt = mean(diff(timestamp))   % average time step
Fs = 1 / dt;

% Print outputs
fprintf('Calculated sample rate Fs: %.2f Hz\n', Fs);

disp('First 5 accelerometer readings:');
disp(accelerometerReadings(1:5, :));

disp('First 5 gyroscope readings:');
disp(gyroscopeReadings(1:5, :));