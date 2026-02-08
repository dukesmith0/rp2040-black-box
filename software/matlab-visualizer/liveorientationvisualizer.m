%% Live Orientation Visualizer
%  Connects to the RP2040 data logger via serial and displays real-time
%  3D orientation using MATLAB's poseplot with quaternion data.
%
%  Serial format expected (from quat_datalogger.ino):
%    ms,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,alt,gps_fix,lat,lon,speed,heading
%
%  Initial text lines from firmware setup are automatically skipped.
%

%% --- Serial Port Selection ---
ports = serialportlist("available");

if isempty(ports)
    error('No serial ports available. Reconnect the device and try again.');
end

fprintf('\nAvailable serial ports:\n');
for i = 1:length(ports)
    fprintf('  [%d] %s\n', i, ports(i));
end

portIdx = input(sprintf('\nSelect port [1-%d]: ', length(ports)));
if isempty(portIdx) || portIdx < 1 || portIdx > length(ports)
    error('Invalid selection.');
end
selectedPort = ports(portIdx);

baudRate = input('Baud rate [115200]: ');
if isempty(baudRate)
    baudRate = 115200;
end

%% --- Connect Serial ---
fprintf('Connecting to %s at %d baud...\n', selectedPort, baudRate);
s = serialport(selectedPort, baudRate);
configureTerminator(s, "LF");
flush(s);

% Brief pause to let firmware finish startup text
pause(1);
flush(s);

fprintf('Connected. Close the figure window to stop.\n\n');

%% --- Setup Poseplot Figure ---
fig = figure('Name', 'Live Orientation', 'NumberTitle', 'off', ...
             'Position', [300 200 700 600]);

p = poseplot(quaternion(1, 0, 0, 0));
set(gca, 'ZDir', 'reverse');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Live Orientation — RP2040 AHRS');

%% --- Live Update Loop ---
while isvalid(fig)
    % Read one line from serial
    if s.NumBytesAvailable == 0
        drawnow limitrate;
        continue;
    end

    line = readline(s);
    line = strip(line);

    if strlength(line) == 0
        continue;
    end

    % Skip non-data lines (setup text, comments, headers)
    firstChar = extractBetween(line, 1, 1);
    if ~ismember(firstChar, ["0","1","2","3","4","5","6","7","8","9"])
        continue;
    end

    % Parse CSV
    values = str2double(split(line, ","));
    if length(values) < 5 || any(isnan(values(1:5)))
        continue;
    end

    % Extract quaternion (columns 2-5: qw, qx, qy, qz)
    % Negate qy and qz to correct frame convention (NED firmware → Z-up display)
    qw = values(2);
    qx = values(3);
    qy = -values(4);
    qz = -values(5);

    % Update poseplot
    set(p, 'Orientation', quaternion(qw, qx, qy, qz));
    drawnow limitrate;
end

%% --- Cleanup on Exit ---
fprintf('Figure closed. Disconnecting serial.\n');
clear s;
