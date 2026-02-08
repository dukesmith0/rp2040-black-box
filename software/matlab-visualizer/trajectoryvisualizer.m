%% Trajectory Visualizer
%  MATLAB recreation of the Streamlit flight_visualizer (app.py)
%  Loads a 20-column CSV from the RP2040 data logger and displays:
%    - Summary statistics in a popup table
%    - 2D GPS track with dead-reckoned segments (green = GPS, orange = DR)
%    - 3D trajectory with dead-reckoned segments
%    - 4-panel sensor charts: Speed & Alt, Quaternion, Accel, Gyro
%
%  Dead reckoning: When gps_fix == 0, body-frame acceleration is rotated
%  to NED using the quaternion, gravity is subtracted, and velocity/position
%  are integrated. Barometric altitude is used for vertical throughout.
%  Position snaps back to GPS when fix returns.
%
%  Requires: Navigation Toolbox (quaternion, rotateframe)
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
% Accidentaly found out that two percent signs create a line in vscode!
opts = detectImportOptions(csvFile, 'CommentStyle', '#');
data = readtable(csvFile, opts);

if ~ismember('ms', data.Properties.VariableNames)
    error('Missing required column: ms');
end

% Check for GPS and quaternion columns (don't filter rows — need all for dead reckoning)
cols = data.Properties.VariableNames;
hasGPS  = all(ismember({'lat','lon'}, cols));
hasQuat = all(ismember({'qw','qx','qy','qz'}, cols));
hasAccel = all(ismember({'ax','ay','az'}, cols));
hasFix  = ismember('gps_fix', cols);

% Build per-row GPS validity flag
if hasGPS && hasFix
    gpsValid = (data.gps_fix == 1) & (data.lat ~= 0) & (data.lon ~= 0);
elseif hasGPS
    gpsValid = (data.lat ~= 0) & (data.lon ~= 0);
else
    gpsValid = false(height(data), 1);
end

t = data.ms / 1000;   % time in seconds

%% --- Compute Fused Trajectory (GPS + Dead Reckoning) ---
% pos_ned: Nx3 array of [North, East, Down] in meters relative to first GPS fix
% posSource: Nx1 logical — true = GPS, false = dead-reckoned

canDeadReckon = hasQuat && hasAccel;
N = height(data);
pos_ned  = zeros(N, 3);
vel_ned  = zeros(N, 3);
posSource = false(N, 1);  % true = GPS

% Reference point: first valid GPS fix
refLat = NaN; refLon = NaN; refAlt = 0;
firstGpsIdx = find(gpsValid, 1, 'first');
if ~isempty(firstGpsIdx)
    refLat = data.lat(firstGpsIdx);
    refLon = data.lon(firstGpsIdx);
    if ismember('alt', cols)
        refAlt = data.alt(firstGpsIdx);
    end
end

hasRef = ~isnan(refLat);

for i = 1:N
    dt = 0;
    if i > 1
        dt = (data.ms(i) - data.ms(i-1)) / 1000.0;  % seconds
        if dt <= 0 || dt > 1.0
            dt = 0.01;  % fallback to 100 Hz nominal if timestamp is bad
        end
    end

    if gpsValid(i) && hasRef
        % --- GPS fix available: use GPS position directly ---
        pos_ned(i, 1) = (data.lat(i) - refLat) * 111000;                     % North
        pos_ned(i, 2) = (data.lon(i) - refLon) * 111000 * cosd(refLat);      % East
        if ismember('alt', cols)
            pos_ned(i, 3) = -(data.alt(i) - refAlt);                          % Down (NED)
        end
        posSource(i) = true;

        % Reset velocity from GPS speed + heading for next DR segment
        if ismember('speed', cols) && ismember('heading', cols)
            spd_ms = data.speed(i) * 0.514444;  % knots to m/s
            hdg_rad = deg2rad(data.heading(i));
            vel_ned(i, :) = [spd_ms * cos(hdg_rad), spd_ms * sin(hdg_rad), 0];
        elseif i > 1
            vel_ned(i, :) = (pos_ned(i, :) - pos_ned(i-1, :)) / dt;
        end

    elseif canDeadReckon && i > 1
        % --- No GPS: dead reckon from IMU ---
        q = quaternion(data.qw(i), data.qx(i), data.qy(i), data.qz(i));
        accel_body = [data.ax(i), data.ay(i), data.az(i)];

        % Rotate body acceleration to NED frame and remove gravity.
        % Sensor at rest reads ~[0, 0, -9.81] (per LSM6DSOX convention).
        % rotateframe rotates body→NED. At rest level: R*[0,0,-9.81] = [0,0,-9.81].
        % Gravity in NED is [0,0,+9.81] (down). Linear accel = measured + [0,0,9.81].
        accel_ned = rotateframe(q, accel_body)' + [0; 0; 9.81];

        % Integrate: velocity and position (trapezoidal would be nicer but
        % Euler is fine at 100 Hz and matches the old filtered_quat_dataplotter)
        vel_ned(i, :) = vel_ned(i-1, :) + accel_ned' * dt;
        pos_ned(i, :) = pos_ned(i-1, :) + vel_ned(i, :) * dt;

        % Use barometric altitude for vertical (much more stable than integration)
        if ismember('alt', cols)
            pos_ned(i, 3) = -(data.alt(i) - refAlt);
        end
        posSource(i) = false;

    elseif i > 1
        % No GPS and can't dead-reckon: hold last position
        pos_ned(i, :) = pos_ned(i-1, :);
        vel_ned(i, :) = [0 0 0];
    end
end

north_m = pos_ned(:, 1);
east_m  = pos_ned(:, 2);
alt_m   = -pos_ned(:, 3);  % Convert NED down back to altitude (up)

% Convert fused NED position back to lat/lon for 2D map plotting
if hasRef
    fusedLat = refLat + north_m / 111000;
    fusedLon = refLon + east_m / (111000 * cosd(refLat));
end

%% --- Compute Summary Statistics ---
duration_s  = (max(data.ms) - min(data.ms)) / 1000;
numSamples  = height(data);

% Distance from fused trajectory (GPS + dead reckoned)
if hasRef && numSamples > 1
    segDist = sqrt(diff(north_m).^2 + diff(east_m).^2);
    totalDist = sum(segDist);
else
    totalDist = NaN;
end

if ismember('speed', cols)
    gpsRows = gpsValid;
    if any(gpsRows)
        avgSpeed = mean(data.speed(gpsRows));
        maxSpeed = max(data.speed(gpsRows));
    else
        avgSpeed = NaN;
        maxSpeed = NaN;
    end
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

%% --- 2D Track (GPS + Dead Reckoning on satellite basemap) ---
% Super sick visualization IMO
if hasRef
    figure('Name', 'Track (GPS + Dead Reckoning)', 'NumberTitle', 'off', ...
           'Position', [150 150 700 550]);

    gx = geoaxes;
    hold(gx, 'on');

    % Thin connecting line for continuity
    geoplot(gx, fusedLat, fusedLon, '-', 'Color', [1 1 1 0.3], ...
            'LineWidth', 1, 'HandleVisibility', 'off');

    % Time-gradient scatter — GPS points (circles) and DR points (diamonds)
    gpsIdx = find(posSource);
    drIdx  = find(~posSource);

    if ~isempty(gpsIdx)
        geoscatter(gx, fusedLat(gpsIdx), fusedLon(gpsIdx), 18, ...
                   t(gpsIdx), 'filled', 'o', 'DisplayName', 'GPS');
    end
    if ~isempty(drIdx)
        geoscatter(gx, fusedLat(drIdx), fusedLon(drIdx), 18, ...
                   t(drIdx), 'filled', 'd', 'DisplayName', 'Dead Reckoned');
    end

    colormap(gx, parula);
    cb = colorbar(gx);
    cb.Label.String = 'Time (s)';

    geobasemap(gx, 'satellite');

    % Start/end markers
    geoscatter(gx, fusedLat(1),   fusedLon(1),   100, ...
               'p', 'MarkerFaceColor', '#22c55e', 'MarkerEdgeColor', 'k', ...
               'DisplayName', 'Start');
    geoscatter(gx, fusedLat(end), fusedLon(end), 100, ...
               'p', 'MarkerFaceColor', '#ef4444', 'MarkerEdgeColor', 'k', ...
               'DisplayName', 'End');

    hold(gx, 'off');
    title(gx, 'Track (GPS + Dead Reckoning)');
    legend(gx, 'Location', 'best');
end

%% --- 3D Trajectory (GPS + Dead Reckoning) ---
if hasRef && ismember('alt', cols)
    figure('Name', '3D Trajectory (GPS + Dead Reckoning)', 'NumberTitle', 'off', ...
           'Position', [200 200 700 550]);
    hold on;

    % Gradient-colored line using the surface trick (degenerate surface = line), thanks to the internet for this
    surface([north_m north_m], [east_m east_m], [alt_m alt_m], [t t], ...
            'FaceColor', 'none', 'EdgeColor', 'interp', 'LineWidth', 3, ...
            'HandleVisibility', 'off');
    colormap(parula);
    cb = colorbar;
    cb.Label.String = 'Time (s)';

    % GPS vs DR markers so source is still visible
    gpsIdx = find(posSource);
    drIdx  = find(~posSource);

    if ~isempty(gpsIdx)
        scatter3(north_m(gpsIdx), east_m(gpsIdx), alt_m(gpsIdx), 12, ...
                 t(gpsIdx), 'filled', 'o', 'MarkerEdgeColor', 'none', ...
                 'DisplayName', 'GPS');
    end
    if ~isempty(drIdx)
        scatter3(north_m(drIdx), east_m(drIdx), alt_m(drIdx), 12, ...
                 t(drIdx), 'filled', 'd', 'MarkerEdgeColor', 'none', ...
                 'DisplayName', 'Dead Reckoned');
    end

    % Start/end markers
    plot3(north_m(1),   east_m(1),   alt_m(1),   'p', 'MarkerSize', 14, ...
          'MarkerFaceColor', '#22c55e', 'MarkerEdgeColor', 'k', ...
          'DisplayName', 'Start');
    plot3(north_m(end), east_m(end), alt_m(end), 'p', 'MarkerSize', 14, ...
          'MarkerFaceColor', '#ef4444', 'MarkerEdgeColor', 'k', ...
          'DisplayName', 'End');

    hold off;
    xlabel('North (m)');
    ylabel('East (m)');
    zlabel('Altitude (m)');
    title('3D Trajectory (GPS + Dead Reckoning)');
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
