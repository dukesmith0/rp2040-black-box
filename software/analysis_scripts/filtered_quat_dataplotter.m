%% RP2040 Black Box Data Analyzer
% Visualizes flight data from the RP2040 data logger.
% Matches the graphs and logic of the Python Streamlit visualizer (app.py).
%
% Features:
%   - Handles CSV files with '# Start:' comment headers
%   - GPS-based trajectory (no dead reckoning drift)
%   - 4-panel sensor charts: Speed/Alt, Quaternion, Accel, Gyro
%   - 3D flight path visualization
%
clear; clc; close all;

% -- Configuration --
useLiveData = false;  % << SET TO true FOR LIVE, false FOR CSV
port = "COM3";
baudrate = 115200;

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

% -- Filter to valid GPS data (like Python visualizer) --
validGPS = (gps_fix == 1) & (lat ~= 0) & (lon ~= 0);
if sum(validGPS) > 0
    hasGPS = true;
    fprintf('Valid GPS samples: %d of %d\n', sum(validGPS), length(validGPS));
else
    hasGPS = false;
    fprintf('No valid GPS data found. Showing sensor data only.\n');
end

% -- Convert GPS to meters from start (same as Python visualizer) --
if hasGPS
    lat_start = lat(find(validGPS, 1));
    lon_start = lon(find(validGPS, 1));
    alt_start = alt(find(validGPS, 1));

    % Convert to meters: lat -> North, lon -> East
    lat_m = (lat - lat_start) * 111000;
    lon_m = (lon - lon_start) * 111000 * cosd(mean(lat(validGPS)));
    alt_m = alt - alt_start;
end

% -- Compute Statistics (like Python visualizer) --
duration = (max(t_s) - min(t_s));
samples = length(t_s);
fprintf('\n--- Statistics ---\n');
fprintf('Duration: %.1f s\n', duration);
fprintf('Samples: %d\n', samples);

if hasGPS
    % Distance calculation (sum of segment lengths)
    lat_diff = diff(lat(validGPS)) * 111000;
    lon_diff = diff(lon(validGPS)) * 111000 * cosd(mean(lat(validGPS)));
    distance = sum(sqrt(lat_diff.^2 + lon_diff.^2));
    fprintf('Distance: %.0f m\n', distance);
    fprintf('Avg Speed: %.1f kts\n', mean(speed(validGPS)));
    fprintf('Max Speed: %.1f kts\n', max(speed(validGPS)));
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

% Subplot 1: Speed & Altitude
ax1 = subplot(2, 2, 1);
hold on;
if hasGPS
    plot(t_s(validGPS), speed(validGPS), 'Color', blue, 'LineWidth', 1, 'DisplayName', 'Speed (kts)');
end
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

% -- Figure 2: 3D Flight Path (GPS-based, like Python) --
if hasGPS
    fig2 = figure('Name', '3D Flight Path', 'Color', [0.15 0.15 0.15]);
    set(fig2, 'Position', [1100, 100, 700, 600]);

    ax3d = axes('Parent', fig2);
    hold(ax3d, 'on');

    % Plot trajectory colored by time using patch (better depth handling)
    validIdx = find(validGPS);
    x = lat_m(validIdx);
    y = lon_m(validIdx);
    z = alt_m(validIdx);
    c = t_s(validIdx);

    % Use patch with edge coloring for proper 3D rendering
    patch([x; nan], [y; nan], [z; nan], [c; nan], ...
          'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', 2);
    colormap(ax3d, 'parula');
    cb = colorbar;
    cb.Label.String = 'Time (s)';
    cb.Color = 'w';

    % Start marker (green)
    plot3(x(1), y(1), z(1), 'o', 'MarkerSize', 12, ...
          'MarkerFaceColor', green, 'MarkerEdgeColor', 'w', 'DisplayName', 'Start');

    % End marker (red)
    plot3(x(end), y(end), z(end), 'o', 'MarkerSize', 12, ...
          'MarkerFaceColor', red, 'MarkerEdgeColor', 'w', 'DisplayName', 'End');

    grid(ax3d, 'on');
    xlabel(ax3d, 'North (m)');
    ylabel(ax3d, 'East (m)');
    zlabel(ax3d, 'Alt (m)');
    title(ax3d, '3D Flight Path', 'Color', 'w');
    view(3);

    % Set equal axis scaling (cube encompassing flight data)
    xRange = max(x) - min(x);
    yRange = max(y) - min(y);
    zRange = max(z) - min(z);
    maxRange = max([xRange, yRange, zRange]);
    if maxRange < 1
        maxRange = 1;  % Prevent zero range
    end

    % Center each axis and apply uniform half-range
    xCenter = (max(x) + min(x)) / 2;
    yCenter = (max(y) + min(y)) / 2;
    zCenter = (max(z) + min(z)) / 2;
    halfRange = maxRange / 2;

    xlim(ax3d, [xCenter - halfRange, xCenter + halfRange]);
    ylim(ax3d, [yCenter - halfRange, yCenter + halfRange]);
    zlim(ax3d, [zCenter - halfRange, zCenter + halfRange]);

    set(ax3d, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    legend(ax3d, 'TextColor', 'w', 'Location', 'best');
end

fprintf('Done.\n');
