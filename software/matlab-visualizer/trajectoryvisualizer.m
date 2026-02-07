%% Trajectory Visualizer
%  MATLAB recreation of the Streamlit flight_visualizer (app.py)
%  Loads a 20-column CSV from the RP2040 data logger and displays:
%    - Summary statistics in a popup table
%    - 2D GPS track (color-coded by time)
%    - 3D trajectory (North / East / Altitude in meters)
%    - 4-panel sensor charts: Speed & Alt, Quaternion, Accel, Gyro
%

%% --- File Selection ---
scriptDir = fileparts(mfilename('fullpath'));
samplesDir = fullfile(scriptDir, '..', '..', 'data', 'samples');

[fileName, filePath] = uigetfile( ...
    {'*.csv', 'CSV Files (*.csv)'}, ...
    'Select a flight data CSV', ...
    samplesDir);

if isequal(fileName, 0)
    disp('No file selected — exiting.');
    return;
end

csvFile = fullfile(filePath, fileName);
fprintf('Loading: %s\n', csvFile);

%% --- Load & Validate CSV ---
% Accidentaly found out that two percent signs create a line!
opts = detectImportOptions(csvFile, 'CommentStyle', '#');
data = readtable(csvFile, opts);

if ~ismember('ms', data.Properties.VariableNames)
    error('Missing required column: ms');
end

% Filter invalid GPS rows
hasGPS = all(ismember({'lat','lon'}, data.Properties.VariableNames));
if hasGPS
    if ismember('gps_fix', data.Properties.VariableNames)
        data = data(data.gps_fix == 1, :);
    end
    data = data(data.lat ~= 0 & data.lon ~= 0, :);
end

cols = data.Properties.VariableNames;
t = data.ms / 1000;   % time in seconds

%% --- Compute Summary Statistics ---
duration_s  = (max(data.ms) - min(data.ms)) / 1000;
numSamples  = height(data);

if hasGPS && numSamples > 1
    latDiff = diff(data.lat) * 111000;
    lonDiff = diff(data.lon) * 111000 * cosd(mean(data.lat));
    totalDist = sum(sqrt(latDiff.^2 + lonDiff.^2));
else
    totalDist = NaN;
end

if ismember('speed', cols)
    avgSpeed = mean(data.speed);
    maxSpeed = max(data.speed);
else
    avgSpeed = NaN;
    maxSpeed = NaN;
end

if ismember('alt', cols)
    altGain = max(data.alt) - min(data.alt);
else
    altGain = NaN;
end

%% --- Show Stats Popup ---
statNames  = {'File'; 'Duration (s)'; 'Samples'; 'Distance (m)'; ...
              'Avg Speed (kts)'; 'Max Speed (kts)'; 'Alt Range (m)'};
statValues = {fileName; sprintf('%.1f', duration_s); ...
              sprintf('%d', numSamples); sprintf('%.1f', totalDist); ...
              sprintf('%.1f', avgSpeed); sprintf('%.1f', maxSpeed); ...
              sprintf('%.1f', altGain)};

statFig = uifigure('Name', 'Flight Summary', ...
    'Position', [100 100 360 250]);
uitable(statFig, ...
    'Data', table(statNames, statValues, ...
                  'VariableNames', {'Metric','Value'}), ...
    'Position', [10 10 340 230], ...
    'ColumnWidth', {140, 180});

%% --- 2D GPS Track (with map basemap!) ---
% The coolest feature here comapared to the Streamlit version.
if hasGPS
    figure('Name', 'GPS Track', 'NumberTitle', 'off', ...
           'Position', [150 150 700 550]);

    gx = geoaxes;
    geoscatter(gx, data.lat, data.lon, 12, t, 'filled');
    geobasemap(gx, 'satellite'); % or 'streets', 'topographic' for different views
    colormap(gx, parula);
    cb = colorbar(gx);
    cb.Label.String = 'Time (s)';
    hold(gx, 'on');
    geoscatter(gx, data.lat(1),   data.lon(1),   100, ...
               'p', 'MarkerFaceColor', '#22c55e', 'MarkerEdgeColor', 'k', ...
               'DisplayName', 'Start');
    geoscatter(gx, data.lat(end), data.lon(end), 100, ...
               'p', 'MarkerFaceColor', '#ef4444', 'MarkerEdgeColor', 'k', ...
               'DisplayName', 'End');
    hold(gx, 'off');
    title(gx, 'GPS Track');
    legend(gx, 'Location', 'best');
end

%% --- 3D Trajectory ---
if hasGPS && ismember('alt', cols)
    lat_m = (data.lat - data.lat(1)) * 111000;
    lon_m = (data.lon - data.lon(1)) * 111000 * cosd(mean(data.lat));
    alt_m = data.alt - data.alt(1);

    figure('Name', '3D Trajectory', 'NumberTitle', 'off', ...
           'Position', [200 200 700 550]);

    scatter3(lat_m, lon_m, alt_m, 12, t, 'filled');
    colormap(parula);
    cb = colorbar;
    cb.Label.String = 'Time (s)';
    hold on;
    plot3(lat_m(1),   lon_m(1),   alt_m(1),   'p', 'MarkerSize', 14, ...
          'MarkerFaceColor', '#22c55e', 'MarkerEdgeColor', 'k', ...
          'DisplayName', 'Start');
    plot3(lat_m(end), lon_m(end), alt_m(end), 'p', 'MarkerSize', 14, ...
          'MarkerFaceColor', '#ef4444', 'MarkerEdgeColor', 'k', ...
          'DisplayName', 'End');
    hold off;
    xlabel('North (m)');
    ylabel('East (m)');
    zlabel('Altitude (m)');
    title('3D Trajectory');
    legend('Location', 'best');
    grid on;
    axis equal;
    view(35, 25);
end

%% --- 4-Panel Sensor Charts ---
figure('Name', 'Sensor Data', 'NumberTitle', 'off', ...
       'Position', [250 100 1100 700]);

% --- Speed & Altitude ---
subplot(2,2,1);
hold on;
if ismember('speed', cols)
    plot(t, data.speed, 'Color', '#3b82f6', 'LineWidth', 1, ...
         'DisplayName', 'Speed (kts)');
end
if ismember('alt', cols)
    plot(t, data.alt, 'Color', '#22c55e', 'LineWidth', 1, ...
         'DisplayName', 'Altitude (m)');
end
hold off;
xlabel('Time (s)');
ylabel('kts / m');
title('Speed & Altitude');
legend('Location', 'best');
grid on;

% --- Quaternion ---
subplot(2,2,2);
hold on;
quatMap = {'qw','#6b7280','w'; 'qx','#ef4444','x'; ...
           'qy','#22c55e','y'; 'qz','#3b82f6','z'};
for k = 1:size(quatMap,1)
    if ismember(quatMap{k,1}, cols)
        plot(t, data.(quatMap{k,1}), 'Color', quatMap{k,2}, ...
             'LineWidth', 1, 'DisplayName', quatMap{k,3});
    end
end
hold off;
xlabel('Time (s)');
ylabel('q');
title('Orientation (Quaternion)');
legend('Location', 'best');
grid on;

% --- Accelerometer ---
subplot(2,2,3);
hold on;
accelMap = {'ax','#ef4444','X'; 'ay','#22c55e','Y'; 'az','#3b82f6','Z'};
for k = 1:size(accelMap,1)
    if ismember(accelMap{k,1}, cols)
        plot(t, data.(accelMap{k,1}), 'Color', accelMap{k,2}, ...
             'LineWidth', 1, 'DisplayName', accelMap{k,3});
    end
end
hold off;
xlabel('Time (s)');
ylabel('m/s²');
title('Accelerometer');
legend('Location', 'best');
grid on;

% --- Gyroscope ---
subplot(2,2,4);
hold on;
gyroMap = {'gx','#ef4444','X'; 'gy','#22c55e','Y'; 'gz','#3b82f6','Z'};
for k = 1:size(gyroMap,1)
    if ismember(gyroMap{k,1}, cols)
        plot(t, data.(gyroMap{k,1}), 'Color', gyroMap{k,2}, ...
             'LineWidth', 1, 'DisplayName', gyroMap{k,3});
    end
end
hold off;
xlabel('Time (s)');
ylabel('rad/s');
title('Gyroscope');
legend('Location', 'best');
grid on;

sgtitle(sprintf('Sensor Data — %s', fileName), 'FontWeight', 'bold');

fprintf('Done. %d samples plotted (%.1f s duration).\n', numSamples, duration_s);
