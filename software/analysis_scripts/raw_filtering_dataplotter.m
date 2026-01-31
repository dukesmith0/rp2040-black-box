%% RP2040 Black Box Raw Data Analyzer with Sensor Fusion
% Processes raw IMU data using MATLAB sensor fusion, then visualizes.
% Handles GPS dropout by dead reckoning from filtered orientation.
%
% Input format (16 columns):
%   ms,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading
%
% Features:
%   - AHRS filtering to compute orientation from raw accel/gyro/mag
%   - Dead reckoning when gps_fix == 0 (extrapolates position from IMU)
%   - GPS-aided trajectory when available
%   - 4-panel sensor charts matching filtered_quat_dataplotter.m
%
% Requires: Navigation Toolbox (for ahrsfilter)
%
clear; clc; close all;

% -- Configuration --
useLiveData = false;  % << SET TO true FOR LIVE, false FOR CSV
port = "COM3";
baudrate = 115200;

% Build path relative to this script's location
scriptDir = fileparts(mfilename('fullpath'));
samplesDir = fullfile(scriptDir, '..', '..', 'data', 'samples');
fileName = fullfile(samplesDir, 'kitti_sample_01_start.csv');

% Sensor parameters (match firmware settings)
sampleRate = 100;  % Hz (LOG_RATE_HZ in raw_datalogger.ino)

% AHRS filter tuning
accelNoise = 0.1;           % Accelerometer noise (m/s^2)
gyroNoise = 0.01;           % Gyroscope noise (rad/s)
magNoise = 0.5;             % Magnetometer noise (uT)
gyroOffset = [0 0 0];       % Gyro bias (rad/s) - calibrate if needed

% -- Data Loading --
if useLiveData
    s = serialport(port, baudrate);
    configureTerminator(s, "LF");
    flush(s);
    fprintf('Streaming... Press Ctrl+C to stop.\n');

    allData = [];
    fprintf('Collecting data... Press Ctrl+C to stop and plot.\n');
    try
        while true
            data = readline(s);
            vals = str2double(split(data, ','));
            if length(vals) >= 16
                allData = [allData; vals'];
            end
            drawnow limitrate;
        end
    catch
        fprintf('Stopped. Collected %d samples.\n', size(allData, 1));
    end
else
    if ~exist(fileName, 'file')
        error('Data file not found: %s\nUpdate the fileName variable.', fileName);
    end
    fprintf('Loading data from %s...\n', fileName);

    % Use CommentStyle to handle '# Start:' header from RTC-enabled firmware
    T = readtable(fileName, 'CommentStyle', '#');
    allData = T{:,:};
    fprintf('Loaded %d samples.\n', size(allData, 1));
end

% -- Extract Raw Data Columns --
% Format: ms,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading
t_ms = allData(:,1);
t_s = t_ms / 1000;
ax = allData(:,2);
ay = allData(:,3);
az = allData(:,4);
gx = allData(:,5);
gy = allData(:,6);
gz = allData(:,7);
mx = allData(:,8);
my = allData(:,9);
mz = allData(:,10);
alt = allData(:,11);
gps_fix = allData(:,12);
lat = allData(:,13);
lon = allData(:,14);
speed_kts = allData(:,15);
heading = allData(:,16);

N = length(t_s);

% -- AHRS Filter Setup --
fprintf('Running AHRS sensor fusion...\n');

% Create AHRS filter (uses Mahony or Madgwick internally)
fuse = ahrsfilter('SampleRate', sampleRate, ...
    'AccelerometerNoise', accelNoise, ...
    'GyroscopeNoise', gyroNoise, ...
    'MagnetometerNoise', magNoise, ...
    'GyroscopeDriftNoise', 1e-6);

% Prepare sensor data matrices
accelData = [ax, ay, az];           % m/s^2
gyroData = [gx, gy, gz];            % rad/s
magData = [mx, my, mz];             % uT (ahrsfilter expects uT)

% Run AHRS filter to get orientation quaternions
[orientation, angularVel] = fuse(accelData, gyroData, magData);

% Extract quaternion components
qw = orientation.a;
qx = orientation.b;
qy = orientation.c;
qz = orientation.d;

fprintf('AHRS filtering complete.\n');

% -- GPS Analysis --
validGPS = (gps_fix == 1) & (lat ~= 0) & (lon ~= 0);
hasAnyGPS = sum(validGPS) > 0;

fprintf('GPS samples: %d valid, %d invalid (will dead reckon)\n', ...
    sum(validGPS), sum(~validGPS));

% -- Position Estimation (GPS + Dead Reckoning) --
fprintf('Computing trajectory with dead reckoning for GPS dropouts...\n');

% Initialize position arrays (in meters from start)
pos_n = zeros(N, 1);  % North
pos_e = zeros(N, 1);  % East
pos_d = zeros(N, 1);  % Down (negative of altitude gain)

% Initialize velocity (m/s)
vel_n = zeros(N, 1);
vel_e = zeros(N, 1);
vel_d = zeros(N, 1);

% Gravity vector in NED frame
g = 9.81;

% Find first valid GPS point for origin
if hasAnyGPS
    firstGPSIdx = find(validGPS, 1);
    lat_origin = lat(firstGPSIdx);
    lon_origin = lon(firstGPSIdx);
    alt_origin = alt(firstGPSIdx);
else
    % No GPS at all - start from origin
    firstGPSIdx = 1;
    lat_origin = 0;
    lon_origin = 0;
    alt_origin = alt(1);
end

% Process each sample
for i = 2:N
    dt = (t_ms(i) - t_ms(i-1)) / 1000;  % Time step in seconds
    if dt <= 0
        dt = 1/sampleRate;  % Fallback
    end

    if validGPS(i)
        % GPS available - use GPS position directly
        pos_n(i) = (lat(i) - lat_origin) * 111000;
        pos_e(i) = (lon(i) - lon_origin) * 111000 * cosd(lat_origin);
        pos_d(i) = -(alt(i) - alt_origin);  % NED: down is positive

        % Estimate velocity from GPS speed and heading
        speed_ms = speed_kts(i) * 0.514444;  % knots to m/s
        vel_n(i) = speed_ms * cosd(heading(i));
        vel_e(i) = speed_ms * sind(heading(i));
        vel_d(i) = 0;  % Assume level flight for GPS velocity
    else
        % No GPS - dead reckon from IMU
        % Get rotation matrix from quaternion
        q = quaternion(qw(i), qx(i), qy(i), qz(i));
        R = rotmat(q, 'frame');  % Body to NED rotation

        % Transform acceleration to NED frame
        accel_body = [ax(i); ay(i); az(i)];
        accel_ned = R * accel_body;

        % Remove gravity (in NED, gravity is [0; 0; g])
        accel_ned(3) = accel_ned(3) - g;

        % Integrate acceleration to velocity
        vel_n(i) = vel_n(i-1) + accel_ned(1) * dt;
        vel_e(i) = vel_e(i-1) + accel_ned(2) * dt;
        vel_d(i) = vel_d(i-1) + accel_ned(3) * dt;

        % Integrate velocity to position
        pos_n(i) = pos_n(i-1) + vel_n(i-1) * dt + 0.5 * accel_ned(1) * dt^2;
        pos_e(i) = pos_e(i-1) + vel_e(i-1) * dt + 0.5 * accel_ned(2) * dt^2;
        pos_d(i) = pos_d(i-1) + vel_d(i-1) * dt + 0.5 * accel_ned(3) * dt^2;

        % Use barometric altitude for vertical (more stable than IMU integration)
        pos_d(i) = -(alt(i) - alt_origin);
    end
end

% Convert position to plotting coordinates
lat_m = pos_n;          % North
lon_m = pos_e;          % East
alt_m = -pos_d;         % Up (negate NED down)

fprintf('Trajectory computation complete.\n');

% -- Compute Statistics --
duration = max(t_s) - min(t_s);
samples = length(t_s);

fprintf('\n--- Statistics ---\n');
fprintf('Duration: %.1f s\n', duration);
fprintf('Samples: %d\n', samples);

% Distance calculation
lat_diff = diff(lat_m);
lon_diff = diff(lon_m);
distance = sum(sqrt(lat_diff.^2 + lon_diff.^2));
fprintf('Total Distance: %.0f m\n', distance);

if hasAnyGPS
    fprintf('GPS-aided segments: %d samples\n', sum(validGPS));
    fprintf('Dead-reckoned segments: %d samples\n', sum(~validGPS));
    fprintf('Avg GPS Speed: %.1f kts\n', mean(speed_kts(validGPS)));
    fprintf('Max GPS Speed: %.1f kts\n', max(speed_kts(validGPS)));
else
    fprintf('Fully dead-reckoned (no GPS data)\n');
end
fprintf('Alt Gain: %.1f m\n', max(alt) - min(alt));
fprintf('------------------\n\n');

% -- Figure 1: Sensor Charts (2x2 layout) --
fig1 = figure('Name', 'Sensor Data (Raw + Filtered)', 'Color', [0.15 0.15 0.15]);
set(fig1, 'Position', [50, 100, 1000, 700]);

% Colors matching Python visualizer
blue = [0.231 0.510 0.965];
green = [0.133 0.773 0.369];
red = [0.937 0.267 0.267];
gray = [0.420 0.455 0.502];
orange = [1.0 0.5 0.0];

% Subplot 1: Speed & Altitude
ax1 = subplot(2, 2, 1);
hold on;
if hasAnyGPS
    plot(t_s(validGPS), speed_kts(validGPS), 'Color', blue, 'LineWidth', 1, 'DisplayName', 'GPS Speed (kts)');
end
% Compute estimated speed from dead reckoning for comparison
vel_mag = sqrt(vel_n.^2 + vel_e.^2) / 0.514444;  % m/s to kts
plot(t_s, vel_mag, 'Color', orange, 'LineWidth', 0.5, 'DisplayName', 'Est Speed (kts)');
plot(t_s, alt, 'Color', green, 'LineWidth', 1, 'DisplayName', 'Altitude (m)');
grid on; xlabel('Time (s)'); ylabel('kts / m');
title('Speed & Altitude', 'Color', 'w');
legend('TextColor', 'w', 'Location', 'best');
set(ax1, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% Subplot 2: Quaternion (from AHRS filter)
ax2 = subplot(2, 2, 2);
hold on;
plot(t_s, qw, 'Color', gray, 'LineWidth', 1, 'DisplayName', 'w');
plot(t_s, qx, 'Color', red, 'LineWidth', 1, 'DisplayName', 'x');
plot(t_s, qy, 'Color', green, 'LineWidth', 1, 'DisplayName', 'y');
plot(t_s, qz, 'Color', blue, 'LineWidth', 1, 'DisplayName', 'z');
grid on; xlabel('Time (s)'); ylabel('q');
title('Orientation (AHRS Filtered)', 'Color', 'w');
legend('TextColor', 'w', 'Location', 'best');
set(ax2, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% Subplot 3: Accelerometer (raw)
ax3 = subplot(2, 2, 3);
hold on;
plot(t_s, ax, 'Color', red, 'LineWidth', 1, 'DisplayName', 'X');
plot(t_s, ay, 'Color', green, 'LineWidth', 1, 'DisplayName', 'Y');
plot(t_s, az, 'Color', blue, 'LineWidth', 1, 'DisplayName', 'Z');
grid on; xlabel('Time (s)'); ylabel('m/s²');
title('Accelerometer (Raw)', 'Color', 'w');
legend('TextColor', 'w', 'Location', 'best');
set(ax3, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% Subplot 4: Gyroscope (raw)
ax4 = subplot(2, 2, 4);
hold on;
plot(t_s, gx, 'Color', red, 'LineWidth', 1, 'DisplayName', 'X');
plot(t_s, gy, 'Color', green, 'LineWidth', 1, 'DisplayName', 'Y');
plot(t_s, gz, 'Color', blue, 'LineWidth', 1, 'DisplayName', 'Z');
grid on; xlabel('Time (s)'); ylabel('rad/s');
title('Gyroscope (Raw)', 'Color', 'w');
legend('TextColor', 'w', 'Location', 'best');
set(ax4, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% -- Figure 2: 3D Flight Path --
fig2 = figure('Name', '3D Flight Path (GPS + Dead Reckoning)', 'Color', [0.15 0.15 0.15]);
set(fig2, 'Position', [1100, 100, 700, 600]);

ax3d = axes('Parent', fig2);
hold(ax3d, 'on');

% Color code by data source: GPS (green) vs Dead Reckoning (orange)
gpsIdx = find(validGPS);
drIdx = find(~validGPS);

% Plot GPS segments
if ~isempty(gpsIdx)
    scatter3(lat_m(gpsIdx), lon_m(gpsIdx), alt_m(gpsIdx), ...
             10, t_s(gpsIdx), 'filled', 'DisplayName', 'GPS');
end

% Plot dead-reckoned segments in different color
if ~isempty(drIdx)
    scatter3(lat_m(drIdx), lon_m(drIdx), alt_m(drIdx), ...
             10, 'MarkerFaceColor', orange, 'MarkerEdgeColor', orange, ...
             'DisplayName', 'Dead Reckoning');
end

colormap(ax3d, 'viridis');
cb = colorbar;
cb.Label.String = 'Time (s)';
cb.Color = 'w';

% Connect all points with line
plot3(lat_m, lon_m, alt_m, 'Color', [1 1 0 0.3], 'LineWidth', 1);

% Start marker (green)
plot3(lat_m(1), lon_m(1), alt_m(1), ...
      'go', 'MarkerSize', 12, 'MarkerFaceColor', green, 'DisplayName', 'Start');

% End marker (red)
plot3(lat_m(end), lon_m(end), alt_m(end), ...
      'ro', 'MarkerSize', 12, 'MarkerFaceColor', red, 'DisplayName', 'End');

grid(ax3d, 'on');
xlabel(ax3d, 'North (m)');
ylabel(ax3d, 'East (m)');
zlabel(ax3d, 'Alt (m)');
title(ax3d, '3D Flight Path (GPS + Dead Reckoning)', 'Color', 'w');
axis(ax3d, 'equal');
view(3);

set(ax3d, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
legend(ax3d, 'TextColor', 'w', 'Location', 'best');

% -- Figure 3: GPS vs Dead Reckoning Comparison (2D top-down) --
fig3 = figure('Name', '2D Track Comparison', 'Color', [0.15 0.15 0.15]);
set(fig3, 'Position', [50, 450, 600, 500]);

ax2d = axes('Parent', fig3);
hold(ax2d, 'on');

% Plot full trajectory
plot(lon_m, lat_m, 'Color', [0.5 0.5 0.5], 'LineWidth', 1, 'DisplayName', 'Full Track');

% Highlight GPS segments
if ~isempty(gpsIdx)
    plot(lon_m(gpsIdx), lat_m(gpsIdx), '.', 'Color', green, 'MarkerSize', 8, 'DisplayName', 'GPS');
end

% Highlight dead-reckoned segments
if ~isempty(drIdx)
    plot(lon_m(drIdx), lat_m(drIdx), '.', 'Color', orange, 'MarkerSize', 8, 'DisplayName', 'Dead Reckoning');
end

% Start/End markers
plot(lon_m(1), lat_m(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', green, 'DisplayName', 'Start');
plot(lon_m(end), lat_m(end), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', red, 'DisplayName', 'End');

grid(ax2d, 'on');
xlabel(ax2d, 'East (m)');
ylabel(ax2d, 'North (m)');
title(ax2d, '2D Track (Top-Down View)', 'Color', 'w');
axis(ax2d, 'equal');

set(ax2d, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
legend(ax2d, 'TextColor', 'w', 'Location', 'best');

fprintf('Done.\n');
