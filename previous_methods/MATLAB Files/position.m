% Load CSV file in the same folder as the script
filename = '2025-08-12 18-07-55.csv';
data = readtable(filename);

% Extract columns
timestamp = data{:,1};  
gyroscopeReadings    = data{:,2:4};
accelerometerReadings= data{:,5:7};

% Unit conversions
accelerometerReadings = accelerometerReadings * 9.80665;
gyroscopeReadings     = deg2rad(gyroscopeReadings);

% Calculate sample rate from timestamps
dt = mean(diff(timestamp))   % average time step
Fs = 1 / dt;

% Print outputs
fprintf('Calculated sample rate Fs: %.2f Hz\n', Fs);

disp('First 5 accelerometer readings:');
disp(accelerometerReadings(1:5, :));

disp('First 5 gyroscope readings:');
disp(gyroscopeReadings(1:5, :));

% Create imufilter object
decim = 1;
fuse = imufilter('SampleRate',Fs,'DecimationFactor',decim);

% Output an estimate of the sensor body orientation over time (quaternions)
q = fuse(accelerometerReadings,gyroscopeReadings);

% Plot the orientation (use actual timestamps)
time = timestamp - timestamp(1);
plot(time,eulerd(q,'ZYX','frame'))
title('Orientation Estimate')
legend('Z-axis', 'Y-axis', 'X-axis')
xlabel('Time (s)')
ylabel('Rotation (degrees)')