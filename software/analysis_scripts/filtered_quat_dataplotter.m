%% RP2040 Black Box Data Analyzer
% Visualizes flight data from the RP2040 data logger.
% Matches the graphs and logic of the Python Streamlit visualizer (app.py).
%
% Features:
%   - Handles CSV files with '# Start:' comment headers
%   - GPS-based trajectory when available (gps_fix == 1)
%   - Dead reckoning when GPS unavailable (gps_fix == 0)
%   - 4-panel sensor charts: Speed/Alt, Quaternion, Accel, Gyro
%   - 3D flight path visualization with GPS/dead-reckoning color coding
%
clear; clc; close all;

% -- Configuration --
useLiveData = false;  % << SET TO true FOR LIVE, false FOR CSV
port = "COM3";
baudrate = 115200;
sampleRate = 100;  % Hz (LOG_RATE_HZ in quat_datalogger.ino)

% Build path relative to this script's location (works regardless of CWD)
scriptDir = fileparts(mfilename('fullpath'));
samplesDir = fullfile(scriptDir, '..', '..', 'data', 'samples');
fileName = fullfile(samplesDir, 'kitti_sample_01_start.csv');

% -- Data Loading --
if useLiveData
    s = serialport(port, baudrate);
    configureTerminator(s, "LF");
    flush(s);
    fprintf('Streaming... Press Ctrl+C to stop.\n');

    % Live mode - collect data then plot
    allData = [];
    fprintf('Collecting data... Press Ctrl+C to stop and plot.\n');
    try
        while true
            data = readline(s);
            vals = str2double(split(data, ','));
            if length(vals) >= 20
                allData = [allData; vals'];
            end
            drawnow limitrate;
        end
    catch
        fprintf('Stopped. Collected %d samples.\n', size(allData, 1));
    end
else
    if ~exist(fileName, 'file')
        error('Data file not found: %s\nUpdate the fileName variable or check the path.', fileName);
    end
    fprintf('Loading data from %s...\n', fileName);

    % Use CommentStyle to handle '# Start:' header from RTC-enabled firmware
    T = readtable(fileName, 'CommentStyle', '#');
    allData = T{:,:};
    fprintf('Loaded %d samples.\n', size(allData, 1));
end

% -- Extract Data Columns --
% Format: ms,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading
t_s = allData(:,1) / 1000;  % Convert ms to seconds
qw = allData(:,2);
qx = allData(:,3);
qy = allData(:,4);
qz = allData(:,5);
ax = allData(:,6);
ay = allData(:,7);
az = allData(:,8);
gx = allData(:,9);
gy = allData(:,10);
gz = allData(:,11);
alt = allData(:,15);
gps_fix = allData(:,16);
lat = allData(:,17);
lon = allData(:,18);
speed = allData(:,19);
heading = allData(:,20);

% -- GPS Analysis --
validGPS = (gps_fix == 1) & (lat ~= 0) & (lon ~= 0);
hasAnyGPS = sum(validGPS) > 0;

fprintf('GPS samples: %d valid, %d invalid (will dead reckon)\n', ...
    sum(validGPS), sum(~validGPS));

% -- Position Estimation (GPS + Dead Reckoning) --
fprintf('Computing trajectory with dead reckoning for GPS dropouts...\n');

N = length(t_s);

% Initialize position arrays (in meters from start)
pos_n = zeros(N, 1);  % North
pos_e = zeros(N, 1);  % East
pos_d = zeros(N, 1);  % Down (negative of altitude gain)

% Initialize velocity (m/s)
vel_n = zeros(N, 1);
vel_e = zeros(N, 1);
vel_d = zeros(N, 1);

% Gravity constant
g = 9.81;

% Find first valid GPS point for origin, or use start position
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
    dt = (t_s(i) - t_s(i-1));  % Time step in seconds
    if dt <= 0
        dt = 1/sampleRate;  % Fallback
    end

    if validGPS(i)
        % GPS available - use GPS position directly
        pos_n(i) = (lat(i) - lat_origin) * 111000;
        pos_e(i) = (lon(i) - lon_origin) * 111000 * cosd(lat_origin);
        pos_d(i) = -(alt(i) - alt_origin);  % NED: down is positive

        % Estimate velocity from GPS speed and heading
        speed_ms = speed(i) * 0.514444;  % knots to m/s
        vel_n(i) = speed_ms * cosd(heading(i));
        vel_e(i) = speed_ms * sind(heading(i));
        vel_d(i) = 0;  % Assume level flight for GPS velocity
    else
        % No GPS - dead reckon from IMU using quaternion orientation
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

% Distance calculation (sum of segment lengths from computed trajectory)
lat_diff = diff(lat_m);
lon_diff = diff(lon_m);
distance = sum(sqrt(lat_diff.^2 + lon_diff.^2));
fprintf('Total Distance: %.0f m\n', distance);

if hasAnyGPS
    fprintf('GPS-aided segments: %d samples\n', sum(validGPS));
    fprintf('Dead-reckoned segments: %d samples\n', sum(~validGPS));
    fprintf('Avg GPS Speed: %.1f kts\n', mean(speed(validGPS)));
    fprintf('Max GPS Speed: %.1f kts\n', max(speed(validGPS)));
else
    fprintf('Fully dead-reckoned (no GPS data)\n');
end
fprintf('Alt Gain: %.1f m\n', max(alt) - min(alt));
fprintf('------------------\n\n');

% -- Figure 1: Sensor Charts (2x2 layout like Python) --
fig1 = figure('Name', 'Sensor Data', 'Color', [0.15 0.15 0.15]);
set(fig1, 'Position', [50, 100, 1000, 700]);

% Colors matching Python visualizer
blue = [0.231 0.510 0.965];    % #3b82f6
green = [0.133 0.773 0.369];   % #22c55e
red = [0.937 0.267 0.267];     % #ef4444
gray = [0.420 0.455 0.502];    % #6b7280
orange = [1.0 0.5 0.0];        % Dead reckoning color

% Subplot 1: Speed & Altitude
ax1 = subplot(2, 2, 1);
hold on;
if hasAnyGPS
    plot(t_s(validGPS), speed(validGPS), 'Color', blue, 'LineWidth', 1, 'DisplayName', 'GPS Speed (kts)');
end
% Compute estimated speed from dead reckoning
vel_mag = sqrt(vel_n.^2 + vel_e.^2) / 0.514444;  % m/s to kts
plot(t_s, vel_mag, 'Color', orange, 'LineWidth', 0.5, 'DisplayName', 'Est Speed (kts)');
plot(t_s, alt, 'Color', green, 'LineWidth', 1, 'DisplayName', 'Altitude (m)');
grid on; xlabel('Time (s)'); ylabel('kts / m');
title('Speed & Altitude', 'Color', 'w');
legend('TextColor', 'w', 'Location', 'best');
set(ax1, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% Subplot 2: Quaternion
ax2 = subplot(2, 2, 2);
hold on;
plot(t_s, qw, 'Color', gray, 'LineWidth', 1, 'DisplayName', 'w');
plot(t_s, qx, 'Color', red, 'LineWidth', 1, 'DisplayName', 'x');
plot(t_s, qy, 'Color', green, 'LineWidth', 1, 'DisplayName', 'y');
plot(t_s, qz, 'Color', blue, 'LineWidth', 1, 'DisplayName', 'z');
grid on; xlabel('Time (s)'); ylabel('q');
title('Orientation (Quaternion)', 'Color', 'w');
legend('TextColor', 'w', 'Location', 'best');
set(ax2, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% Subplot 3: Accelerometer
ax3 = subplot(2, 2, 3);
hold on;
plot(t_s, ax, 'Color', red, 'LineWidth', 1, 'DisplayName', 'X');
plot(t_s, ay, 'Color', green, 'LineWidth', 1, 'DisplayName', 'Y');
plot(t_s, az, 'Color', blue, 'LineWidth', 1, 'DisplayName', 'Z');
grid on; xlabel('Time (s)'); ylabel('m/s²');
title('Accelerometer', 'Color', 'w');
legend('TextColor', 'w', 'Location', 'best');
set(ax3, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% Subplot 4: Gyroscope
ax4 = subplot(2, 2, 4);
hold on;
plot(t_s, gx, 'Color', red, 'LineWidth', 1, 'DisplayName', 'X');
plot(t_s, gy, 'Color', green, 'LineWidth', 1, 'DisplayName', 'Y');
plot(t_s, gz, 'Color', blue, 'LineWidth', 1, 'DisplayName', 'Z');
grid on; xlabel('Time (s)'); ylabel('rad/s');
title('Gyroscope', 'Color', 'w');
legend('TextColor', 'w', 'Location', 'best');
set(ax4, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% -- Figure 2: 3D Flight Path (GPS + Dead Reckoning) --
fig2 = figure('Name', '3D Flight Path (GPS + Dead Reckoning)', 'Color', [0.15 0.15 0.15]);
set(fig2, 'Position', [1100, 100, 700, 600]);

ax3d = axes('Parent', fig2);
hold(ax3d, 'on');

% Color code by data source: GPS vs Dead Reckoning
gpsIdx = find(validGPS);
drIdx = find(~validGPS);

% Plot GPS segments (colored by time)
if ~isempty(gpsIdx)
    scatter3(lat_m(gpsIdx), lon_m(gpsIdx), alt_m(gpsIdx), ...
             10, t_s(gpsIdx), 'filled', 'DisplayName', 'GPS');
end

% Plot dead-reckoned segments in orange
if ~isempty(drIdx)
    scatter3(lat_m(drIdx), lon_m(drIdx), alt_m(drIdx), ...
             10, 'MarkerFaceColor', orange, 'MarkerEdgeColor', orange, ...
             'DisplayName', 'Dead Reckoning');
end

colormap(ax3d, 'parula');
cb = colorbar;
cb.Label.String = 'Time (s)';
cb.Color = 'w';

% Connect all points with line
plot3(lat_m, lon_m, alt_m, 'Color', [1 1 0 0.3], 'LineWidth', 1);

% Start marker (green)
plot3(lat_m(1), lon_m(1), alt_m(1), 'o', 'MarkerSize', 12, ...
      'MarkerFaceColor', green, 'MarkerEdgeColor', 'w', 'DisplayName', 'Start');

% End marker (red)
plot3(lat_m(end), lon_m(end), alt_m(end), 'o', 'MarkerSize', 12, ...
      'MarkerFaceColor', red, 'MarkerEdgeColor', 'w', 'DisplayName', 'End');

grid(ax3d, 'on');
xlabel(ax3d, 'North (m)');
ylabel(ax3d, 'East (m)');
zlabel(ax3d, 'Alt (m)');
title(ax3d, '3D Flight Path (GPS + Dead Reckoning)', 'Color', 'w');
view(3);

% Set equal axis scaling
xRange = max(lat_m) - min(lat_m);
yRange = max(lon_m) - min(lon_m);
zRange = max(alt_m) - min(alt_m);
maxRange = max([xRange, yRange, zRange]);
if maxRange < 1
    maxRange = 1;  % Prevent zero range
end

xCenter = (max(lat_m) + min(lat_m)) / 2;
yCenter = (max(lon_m) + min(lon_m)) / 2;
zCenter = (max(alt_m) + min(alt_m)) / 2;
halfRange = maxRange / 2;

xlim(ax3d, [xCenter - halfRange, xCenter + halfRange]);
ylim(ax3d, [yCenter - halfRange, yCenter + halfRange]);
zlim(ax3d, [zCenter - halfRange, zCenter + halfRange]);

set(ax3d, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
legend(ax3d, 'TextColor', 'w', 'Location', 'best');

% -- Figure 3: 2D Track (Top-Down View) --
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
