%% RP2040 Adalogger Data Analyzer - Z-Up NEU Frame
clear; clc; close all;

% -- Configuration --
useLiveData = false; % << SET TO true FOR LIVE, false FOR CSV
port = "COM3";
baudrate = 115200;
g = 9.80665;
fileName = "test_data.csv";

% -- Data Loading --
if useLiveData
    s = serialport(port, baudrate);
    configureTerminator(s, "LF");
    flush(s);
    fprintf('Streaming... Press Ctrl+C to stop.\n');
    allData = []; % Not pre-loaded for live
else
    if ~exist(fileName, 'file')
        error('Specified data file not found: %s', fileName);
    end
    fprintf('Loading data from %s...\n', fileName);
    allData = readmatrix(fileName);
    s = []; % No serial object
end

% -- Figure Setup --
fig = figure('Name', 'Sensor Fusion Analysis (NEU Frame)', 'Color', [0.15 0.15 0.15]);
set(fig, 'Position', [100, 100, 1200, 800]);

% Subplot 1: Position (X-Y)
axPos = subplot(3, 1, 1);
hX = animatedline('Color', 'c', 'LineWidth', 1.5, 'DisplayName', 'North (X)');
hY = animatedline('Color', 'm', 'LineWidth', 1.5, 'DisplayName', 'East (Y)');
grid on; ylabel('Position (m)'); legend('TextColor', 'w');
title('Global Position (Local NEU)', 'Color', 'w'); set(axPos, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% Subplot 2: Altitude (Z-Up)
axAlt = subplot(3, 1, 2);
hAlt = animatedline('Color', 'g', 'LineWidth', 1.5);
grid on; ylabel('Up (m)');
title('Altitude (Z-Up)', 'Color', 'w'); set(axAlt, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% Subplot 3: Orientation (Euler Angles)
axOri = subplot(3, 1, 3);
hRoll  = animatedline('Color', [0 1 0], 'DisplayName', 'Roll');
hPitch = animatedline('Color', [1 0.5 0], 'DisplayName', 'Pitch');
hYaw   = animatedline('Color', [0 0.5 1], 'DisplayName', 'Yaw');
grid on; ylabel('Angle (deg)'); xlabel('Time (ms)'); legend('TextColor', 'w');
title('Attitude (Euler Angles)', 'Color', 'w'); set(axOri, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% -- 3D Flight Path Figure --
fig3d = figure('Name', '3D Flight Path (NEU)', 'Color', [0.15 0.15 0.15]);
set(fig3d, 'Position', [1350, 100, 800, 800]); % Position it next to the other figure
ax3d = axes('Parent', fig3d, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
hold(ax3d, 'on');
grid(ax3d, 'on');
axis(ax3d, 'equal');
xlabel(ax3d, 'North (X) (m)');
ylabel(ax3d, 'East (Y) (m)');
zlabel(ax3d, 'Up (Z) (m)');
title(ax3d, '3D Flight Path', 'Color', 'w');
view(3); % Standard 3D view
hPath = animatedline(ax3d, 'Color', 'y', 'LineWidth', 2, 'MaximumNumPoints', 20000);

% Plot home position
plot3(ax3d, 0, 0, 0, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Home');
legend(ax3d, 'TextColor', 'w');

% -- State Variables --
pos_NEU = [0; 0; 0];
vel_NEU = [0; 0; 0];
isInitialized = false;
startTime = 0;
alt_home = 0;

% Quaternion to rotate from NED (filter output) to NEU (our desired frame)
% This is a 180-degree rotation around the X (North) axis.
q_NED_to_NEU = quaternion([180, 0, 0], 'eulerd', 'XYZ', 'frame');

% -- Main Processing Loop --
loopCondition = true;
i = 1;
while loopCondition
    try
        if useLiveData
            data = readline(s);
            vals = str2double(split(data, ','));
        else
            if i > size(allData, 1), break; end
            vals = allData(i,:);
            i = i + 1;
        end

        if length(vals) < 16, continue; end % Header has 16 columns

        % Data Mapping
        t_ms = vals(1);
        q_raw_NED = quaternion(vals(2), vals(3), vals(4), vals(5)); % w, x, y, z from filter is NED
        accel_body = [vals(6); vals(7); vals(8)]; % Raw sensor data is Z-up
        baro_alt = vals(15);

        if ~isInitialized
            startTime = t_ms;
            alt_home = baro_alt;
            isInitialized = true;
        end
        rel_time = t_ms - startTime;

        % 1. Convert incoming NED quaternion to NEU
        q_NEU = q_NED_to_NEU * q_raw_NED;
        
        % 2. Convert Quaternion to Euler (Degrees) for plotting
        euler = eulerd(q_NEU, 'ZYX', 'frame'); % Yaw, Pitch, Roll

        % 3. Transform Accel to NEU Frame for integration
        % Rotate body accel to world frame and add gravity vector (since sensor measures reaction force)
        % Gravity in NEU is [0, 0, -g]
        accel_NEU = rotatepoint(q_NEU, accel_body')' + [0; 0; -g];

        % 4. Simple Integration for Position (Dead Reckoning)
        dt = 0.01; % Ideal 10ms loop time
        vel_NEU = vel_NEU + accel_NEU * dt;
        pos_NEU = pos_NEU + vel_NEU * dt;

        % 5. Altitude (Z-Up)
        z_up = baro_alt - alt_home;

        % Update Plots
        addpoints(hX, rel_time, pos_NEU(1));
        addpoints(hY, rel_time, pos_NEU(2));
        addpoints(hAlt, rel_time, z_up);
        addpoints(hRoll, rel_time, euler(3));
        addpoints(hPitch, rel_time, euler(2));
        addpoints(hYaw, rel_time, euler(1));
        addpoints(hPath, pos_NEU(1), pos_NEU(2), z_up);

        % Auto-scroll X-Axis
        if useLiveData && rel_time > 10000
             xlim(axPos, [rel_time-10000, rel_time]);
             xlim(axAlt, [rel_time-10000, rel_time]);
             xlim(axOri, [rel_time-10000, rel_time]);
        end

        drawnow limitrate;

    catch ME
        fprintf('An error occurred: %s\n', ME.message);
        if useLiveData, continue; else, break; end
    end
    
    if ~useLiveData && i > size(allData, 1)
        loopCondition = false;
        fprintf('Finished processing %s.\n', fileName);
    end
end