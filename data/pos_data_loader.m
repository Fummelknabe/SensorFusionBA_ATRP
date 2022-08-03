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
% Get Camera Position (note that position is being flipped)
camPos = zeros(l, 3);
for i=1:l
    camPos(i, 1) = val(i).cameraPos(1);
    camPos(i, 2) = -val(i).cameraPos(3);
    camPos(i, 3) = val(i).cameraPos(2);
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
    % Use acceleration data
    %downVector = [0; -1; 0] .* acc(i);
    %northVector = val(i).imuMag - (downVector * (dot(val(i).imuMag, downVector) / dot(downVector, downVector)));
    %angle = atan2(northVector(1), northVector(3));
    angle = atan2(val(i).imuMag(1), val(i).imuMag(2));
    if angle > 0
        phi(i) = angle;
    else
        phi(i) = angle + 2*pi;
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

steerAngle = zeros(l, 1);
for i=1:l
    steerAngle(i) = (val(i).steerAngle - 120) * 0.075; 
end

%% Camera Confidence
camConf = zeros(l, 1);
for i=1:l
    % Convert to [0, 1]
    camConf(i) = val(i).cameraConfidence / 100;
end

%% Camera Orientation
rotMatrix = zeros(l, 4, 4);
eulerAngles = zeros(l, 3);
for i=1:l
    % Rotation Matrix from Quaternion
    x = val(i).cameraOri(1);
    y = val(i).cameraOri(2);
    z = val(i).cameraOri(3);
    w = val(i).cameraOri(4);
    rotMatrix(i, :, :) = [1-2*y^2-2*z^2, 2*x*y-2*w*z, 2*x*z + 2*w*y, 0;
                          2*x*y+2*w*z, 1-2*x^2-2*z^2, 2*y*z-2*w*x, 0;
                          2*x*z-2*w*y, 2*y*z+2*x*w, 1-2*x^2-2*y^2, 0;
                          0, 0, 0, 1];
                      
    % Euler Angles from Quaternion
    eulerAngles(i, :) = [atan2(2*(w*x+y*z), 1-2*(x^2+y^2)); asin(2*(w*y-z*x)); atan2(2*(w*z+x*y), 1-2*(y^2+z^2))];
end
clear x y z w rotMatrix

%% Odometry (Angular Velocity)
odometryPosAngVel = zeros(l, 3);
odometryPosAngVel(1, :) = camPos(1, :); % inital position
oldPhi = phi(1);

for i=2:l
    % With Angular Velocity:
    phi_dot = angVel(i, 3);
    
    % Get change in position 
    x_dot = v(i) * cos(oldPhi - dt(i)*phi_dot); 
    y_dot = v(i) * sin(oldPhi - dt(i)*phi_dot);  
    oldPhi = (oldPhi - dt(i)*phi_dot); 
   
    % predict 
    odometryPosAngVel(i, 1) = odometryPosAngVel(i-1, 1) + x_dot * dt(i);% x-Pos
    odometryPosAngVel(i, 2) = odometryPosAngVel(i-1, 2) + y_dot * dt(i);% y-Pos
    odometryPosAngVel(i, 3) = odometryPosAngVel(i-1, 3); % z-Pos
end

%% Odometry (Steering Angle)
odometryPosSteAng = zeros(l, 3);
odometryPosSteAng(1, :) = camPos(1, :); % inital position
oldPhi = phi(1);

for i=2:l        
    % With Steering Angle: 
    x_dot = v(i) * cos(oldPhi + dt(i)*steerAngle(i)); 
    y_dot = v(i) * sin(oldPhi + dt(i)*steerAngle(i));  
    oldPhi = (oldPhi + dt(i)*steerAngle(i));
    
    % predict 
    odometryPosSteAng(i, 1) = odometryPosSteAng(i-1, 1) + x_dot * dt(i);% x-Pos
    odometryPosSteAng(i, 2) = odometryPosSteAng(i-1, 2) + y_dot * dt(i);% y-Pos
    odometryPosSteAng(i, 3) = odometryPosSteAng(i-1, 3); % z-Pos
end

%% Odometry (Compass Course)
odometryPosCompass = zeros(l, 3);
odometryPosCompass(1, :) = camPos(1, :); % inital position

for i=2:l        
    % With Compass Course:
    x_dot = v(i) * cos(phi(i));
    y_dot = v(i) * sin(phi(i));  
    
    % predict 
    odometryPosCompass(i, 1) = odometryPosCompass(i-1, 1) + x_dot * dt(i);% x-Pos
    odometryPosCompass(i, 2) = odometryPosCompass(i-1, 2) + y_dot * dt(i);% y-Pos
    odometryPosCompass(i, 3) = odometryPosCompass(i-1, 3); % z-Pos
end

%% Odometry (Camera Orientation)
odometryPosCamera = zeros(l, 3);
odometryPosCamera(1, :) = camPos(1, :); % inital position

for i=2:l        
    % With Camera Orientation: 
    y_dot = v(i) * cos(eulerAngles(i, 2));
    x_dot = -v(i) * sin(eulerAngles(i, 2)); 
    
    % predict 
    odometryPosCamera(i, 1) = odometryPosCamera(i-1, 1) + x_dot * dt(i);% x-Pos
    odometryPosCamera(i, 2) = odometryPosCamera(i-1, 2) + y_dot * dt(i);% y-Pos
    odometryPosCamera(i, 3) = odometryPosCamera(i-1, 3); % z-Pos
end

%% Predicted Position
predictedPos = zeros(l, 3);
predictedPos(1, :) = camPos(1, :); % inital position (the same as odometry position)%

%for i=2:l
%    deltaOdometry = odometryPos(i, :) - odometryPos(i - 1, :);
%    deltaComPos = camPos(i, :) - camPos(i - 1, :);
%    predictedPos(i, :) = predictedPos(i-1, :) + ((1 - camConf(i)) .* deltaOdometry + camConf(i) .* deltaComPos);
%end

for i=2:l
    deltaSteAng = odometryPosSteAng(i, :) - odometryPosSteAng(i - 1, :);
    deltaAngVel = odometryPosAngVel(i, :) - odometryPosAngVel(i - 1, :);
    predictedPos(i, :) = predictedPos(i-1, :) + (deltaAngVel + deltaSteAng)./2;
end

%% Plot Position 
subplot(1, 2, 1)
% Plot camera position in 3D
%scatter3(camPos(:, 1), camPos(:, 2), camPos(:, 3))
plot(1:l, phi .* 180/pi)
%title('Camera Position in 3D')

% Plot camera position in 2D
subplot(1, 2, 2)
scatter(camPos(:, 1), camPos(:, 2), 'b')
hold on
scatter(odometryPosCamera(:, 1), odometryPosCamera(:, 2), 'm')
hold on
scatter(odometryPosCompass(:, 1), odometryPosCompass(:, 2), 'r')
hold on
scatter(odometryPosAngVel(:, 1), odometryPosAngVel(:, 2), 'c')
hold on
scatter(odometryPosSteAng(:, 1), odometryPosSteAng(:, 2), 'g')
hold on
scatter(predictedPos(:, 1), predictedPos(:, 2), 'y')
title('Cam vs Odometry in 2D')
