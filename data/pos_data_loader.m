clc
clear

% Load Json file
filename = 'pos_data.json';
fid = fopen(filename); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
val = jsondecode(str);
l = length(val);

clear raw str

%% Camera Position
% Get Camera Position
camPos = zeros(l, 3);
for i=1:l
    camPos(i, 1) = val(i).cameraPos(1);
    camPos(i, 2) = val(i).cameraPos(2);
    camPos(i, 3) = val(i).cameraPos(3);
end

%% Acceleration Data
acc = zeros(l, 3);
for i=1:801
    acc(i, :) = val(i).imuAcc;
end

%% Magnetometer Data
mag = zeros(l, 3);
for i=1:l
    mag(i, :) = val(i).imuMag;
end

%% Compass Course
phi = zeros(l, 1);
for i=1:l
    downVector = [0; -1; 0] .* acc(i);
    northVector = val(i).imuMag - (downVector * (dot(val(i).imuMag, downVector) / dot(downVector, downVector)));
    angle = atan2(northVector(1), northVector(3));
    if angle > 0
        phi(i) = angle;
    else
        phi(i) = angle + pi/2;
    end
end

%% Velocity
v = zeros(l, 1);
for i=1:l
    v(i) = val(i).sensorSpeed;
end

%% Delta Time
dt = zeros(l, 1);
for i=1:l
    dt(i) = val(i).deltaTime;
end

%% Angular Velocity 
angVel = zeros(l, 3);
for i=1:l
    % Convert to rad/s as data is given in °/s
    angVel(i, :) = val(i).imuGyro .* pi/180;
end

%% Camera Confidence
camConf = zeros(l, 1);
for i=1:l
    % Convert to [0, 1]
    camConf(i) = val(i).cameraConfidence / 100;
end

%% Odometry
odometryPos = zeros(l, 3);
odometryPos(1, :) = camPos(1, :); % inital position
oldPhi = phi(1);
oldPhiDot = angVel(1);

for i=2:l
    % This should be used to correct prediction:
    if dt(i) < 0.5
        phi_dot = angVel(i);
        oldPhiDot = angVel(i);
    else
        phi_dot = oldPhiDot;
    end
    
    % Get change in position
    x_dot = v(i) * cos((oldPhi + dt(i)*phi_dot)/2); % phi(i)
    y_dot = v(i) * sin((oldPhi + dt(i)*phi_dot)/2); % phi(i)   
    oldPhi = (oldPhi + dt(i)*phi_dot)/2;
    
    % predict
    odometryPos(i, 1) = odometryPos(i-1, 1) + x_dot * dt(i);% x-Pos
    odometryPos(i, 2) = odometryPos(i-1, 2) + y_dot * dt(i);% y-Pos
    odometryPos(i, 3) = odometryPos(i-1, 3); % z-Pos
end

%% Predicted Position
predictedPos = zeros(l, 3);
predictedPos(1, :) = camPos(1, :); % inital position (the same as odometry position)

for i=2:l
    deltaOdometry = odometryPos(i, :) - odometryPos(i - 1, :);
    deltaComPos = camPos(i, :) - camPos(i - 1, :);
    predictedPos(i, :) = predictedPos(i-1, :) + ((1 - camConf(i)) .* deltaOdometry + camConf(i) .* deltaComPos);
end

%% Plot Position 
subplot(1, 2, 1)
% Plot camera position in 3D
scatter3(camPos(:, 1), camPos(:, 2), camPos(:, 3))
axis([60 100 -10 30 -10 30 ])
title('Camera Position in 3D')

% Plot camera position in 2D
subplot(1, 2, 2)
scatter(camPos(:, 1), camPos(:, 2), 'b')
hold on
scatter(odometryPos(:, 1), odometryPos(:, 2), 'g')
hold on
scatter(predictedPos(:, 1), predictedPos(:, 2), 'r')
title('Cam vs Prediction vs Odometry in 2D')
