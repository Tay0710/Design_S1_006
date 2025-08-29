%% Housekeeping
addpath('ximu_matlab_library');   % x-IMU MATLAB library (MahonyAHRS, etc.)
addpath('quaternion_library');    % quaternion helpers (quatern2rotMat)
close all; clear; clc;

%% Import data from your CSV
% !!! Update this to the file you want to use:
csvFile = 'LoggedData/2025-08-08 19-20-23.csv';

% Read with original headers preserved; we'll resolve robustly anyway
T = readtable(csvFile, 'VariableNamingRule', 'preserve');

% --- Resolve columns by label or any MATLAB-modified variant ---
time = getVar(T, "Time (s)");
gx   = getVar(T, "Gyroscope X (deg/s)");
gy   = getVar(T, "Gyroscope Y (deg/s)");
gz   = getVar(T, "Gyroscope Z (deg/s)");
ax   = getVar(T, "Accelerometer X (g)");
ay   = getVar(T, "Accelerometer Y (g)");
az   = getVar(T, "Accelerometer Z (g)");

% Build arrays
gyr = [gx gy gz];
acc = [ax ay az];

% Units and timing
gyroUnits    = 'deg/s';       % your gyro units
accUnits     = 'g';           % your accel units
samplePeriod = mean(diff(time));   % seconds/sample

%% AHRS (Mahony) – orientation (sensor -> Earth)
R = zeros(3,3,length(gyr));
ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);

for i = 1:length(gyr)
    if strcmpi(gyroUnits,'deg/s')
        g_in = gyr(i,:) * (pi/180);   % deg/s -> rad/s
    else
        g_in = gyr(i,:);
    end
    ahrs.UpdateIMU(g_in, acc(i,:));              % accel in g
    R(:,:,i) = quatern2rotMat(ahrs.Quaternion)'; % Earth<-sensor
end

%% Tilt-compensated accelerometer (Earth frame, still in g)
tcAcc = zeros(size(acc));
for i = 1:length(acc)
    tcAcc(i,:) = (R(:,:,i) * acc(i,:)')';
end

%% Linear acceleration in Earth frame (m/s^2): subtract gravity on Z, convert g -> m/s^2
linAcc_g = tcAcc - [zeros(size(tcAcc,1),2) ones(size(tcAcc,1),1)];
linAcc   = linAcc_g * 9.81;

%% Integrate to velocity (m/s)
linVel = zeros(size(linAcc));
for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;
end

%% High-pass filter velocity (remove drift)
order = 1;           % 1st order
filtCutOff = 0.1;    % Hz (tune as needed)
[b,a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linVelHP = filtfilt(b, a, linVel);

%% Integrate to position (m)
linPos = zeros(size(linVelHP));
for i = 2:length(linVelHP)
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * samplePeriod;
end

%% High-pass filter position (optional extra cleanup)
[b2,a2] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linPosHP = filtfilt(b2, a2, linPos);

%% ===== Combined figure with subplots (all plots except animation) =====
f = figure('Name','IMU Processing Results','Position',[100 100 1200 900]);
t = tiledlayout(3,3,'TileSpacing','compact','Padding','compact');

% 1) Gyroscope
nexttile;
plot(gyr(:,1),'r'); hold on; plot(gyr(:,2),'g'); plot(gyr(:,3),'b'); grid on;
xlabel('Sample'); ylabel('deg/s'); legend('X','Y','Z','Location','best'); title('Gyroscope');

% 2) Accelerometer (g)
nexttile;
plot(acc(:,1),'r'); hold on; plot(acc(:,2),'g'); plot(acc(:,3),'b'); grid on;
xlabel('Sample'); ylabel('g'); legend('X','Y','Z','Location','best'); title('Accelerometer');

% 3) Tilt-Compensated Accel (g)
nexttile;
plot(tcAcc(:,1),'r'); hold on; plot(tcAcc(:,2),'g'); plot(tcAcc(:,3),'b'); grid on;
xlabel('Sample'); ylabel('g'); legend('X','Y','Z','Location','best'); title('Tilt-Comp Accel');

% 4) Linear Acceleration (m/s^2)
ax1 = nexttile; %#ok<NASGU>
plot(linAcc(:,1),'r'); hold on; plot(linAcc(:,2),'g'); plot(linAcc(:,3),'b'); grid on;
xlabel('Sample'); ylabel('m/s^2'); legend('X','Y','Z','Location','best'); title('Linear Accel');

% 5) Linear Velocity (m/s)
ax2 = nexttile; %#ok<NASGU>
plot(linVel(:,1),'r'); hold on; plot(linVel(:,2),'g'); plot(linVel(:,3),'b'); grid on;
xlabel('Sample'); ylabel('m/s'); legend('X','Y','Z','Location','best'); title('Linear Velocity');

% 6) HPF Velocity (m/s)
ax3 = nexttile; %#ok<NASGU>
plot(linVelHP(:,1),'r'); hold on; plot(linVelHP(:,2),'g'); plot(linVelHP(:,3),'b'); grid on;
xlabel('Sample'); ylabel('m/s'); legend('X','Y','Z','Location','best'); title('HPF Velocity');

% 7) Linear Position (m)
ax4 = nexttile; %#ok<NASGU>
plot(linPos(:,1),'r'); hold on; plot(linPos(:,2),'g'); plot(linPos(:,3),'b'); grid on;
xlabel('Sample'); ylabel('m'); legend('X','Y','Z','Location','best'); title('Linear Position');

% 8) HPF Position (m)
ax5 = nexttile; %#ok<NASGU>
plot(linPosHP(:,1),'r'); hold on; plot(linPosHP(:,2),'g'); plot(linPosHP(:,3),'b'); grid on;
xlabel('Sample'); ylabel('m'); legend('X','Y','Z','Location','best'); title('HPF Position');

% 9) 2D XY Path
nexttile;
plot(linPosHP(:,1), linPosHP(:,2), '-'); axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)'); title('XY Trajectory');

title(t, 'IMU → AHRS → Tilt-Comp → Gravity Removal → HPF → Integrate', 'FontWeight','bold');

% Link X-axes for the time-series plots (tiles 1..8)
axs = findall(f,'Type','axes');
linkaxes(axs(1:8),'x');  % last tile is XY so we leave it out

%% Animation (show full trail of all previous positions)
SamplePlotFreq = 1;
try
    % Try with explicit full-length trail (if your SixDOFanimation supports it)
    SixDOFanimation(linPosHP, R, ...
        'SamplePlotFreq', SamplePlotFreq, ...
        'Trail', 'All', ...                          % show trail
        'TrailLength', size(linPosHP,1), ...        % full history
        'Position', [20 60 1280 720], ...
        'AxisLength', 0.1, 'ShowArrowHead', false, ...
        'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', ...
        'ShowLegend', false, 'Title', 'Unfiltered', ...
        'CreateAVI', false, 'AVIfileNameEnum', false, ...
        'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
catch
    % Fallback for versions that only accept 'Trail' On/Off
    SixDOFanimation(linPosHP, R, ...
        'SamplePlotFreq', SamplePlotFreq, ...
        'Trail', 'All', ...
        'Position', [20 60 1280 720], ...
        'AxisLength', 0.1, 'ShowArrowHead', false, ...
        'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', ...
        'ShowLegend', false, 'Title', 'Unfiltered', ...
        'CreateAVI', false, 'AVIfileNameEnum', false, ...
        'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
end

%% ==== Local helper: tolerant variable resolver ====
function v = getVar(T, target)
    % Try exact match first (preserved headers)
    vars = T.Properties.VariableNames;
    ix = find(strcmp(vars, target), 1);
    if ~isempty(ix), v = T.(vars{ix}); return; end

    % Try MATLAB's sanitized version of the target
    san = matlab.lang.makeValidName(target, 'ReplacementStyle', 'underscore');
    ix = find(strcmp(vars, san), 1);
    if ~isempty(ix), v = T.(vars{ix}); return; end

    % Case-insensitive exact
    ix = find(strcmpi(vars, target), 1);
    if ~isempty(ix), v = T.(vars{ix}); return; end

    % Case-insensitive sanitized
    ix = find(strcmpi(vars, san), 1);
    if ~isempty(ix), v = T.(vars{ix}); return; end

    % Loose contains-match (strip non-alphanumerics)
    scrub = @(s) lower(regexprep(string(s), '[^a-zA-Z0-9]', ''));
    sv = scrub(vars);
    st = scrub(target);
    ix = find(contains(sv, st), 1);
    if ~isempty(ix), v = T.(vars{ix}); return; end

    % If still not found, show user what exists
    error('Could not find column for "%s". Available columns:\n  %s', ...
        target, strjoin(vars, ', '));
end
