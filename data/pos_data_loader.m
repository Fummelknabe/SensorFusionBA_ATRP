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

rateCamConf = @(x) x^4;

%% Delta Time
dt = zeros(l, 1);
for i=1:l
    dt(i) = val(i).deltaTime;
end

%% Camera Position
% Get Camera Position (note that position is being flipped)
camPos = zeros(l, 3);
camChange = zeros(l, 1);
for i=1:l
    camPos(i, 1) = val(i).cameraPos(1);
    camPos(i, 2) = -val(i).cameraPos(3);
    camPos(i, 3) = val(i).cameraPos(2);
    camChange(i) = val(i).cameraPos(4);
end

%% Camera Speed
cameraSpeed = camChange ./ dt;
gauss_filter = gausswin(15, 1.5);
gauss_filter = gauss_filter/sum(gauss_filter);
cameraSpeed = filter(gauss_filter, 1, cameraSpeed);

%% Camera Confidence
camConf = zeros(l, 1);
for i=1:l
    % Convert to [0, 1]
    camConf(i) = val(i).cameraConfidence / 100;
end

%% Acceleration Data
acc = zeros(l, 3);
for i=1:l
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

%% Combined Velocity
v_combined = zeros(l, 1);

for i=1:l
    v_combined(i) = (1-rateCamConf(camConf(i)))*v(i) + rateCamConf(camConf(i))*cameraSpeed(i);
end
% filter results:
gauss_filter = gausswin(8, 2.5);
gauss_filter = gauss_filter/sum(gauss_filter);
v_combined = filter(gauss_filter, 1, v_combined);

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

%% Initial Up Angle (taken from acceleration data with v = 0)
initUpVector = acc(i, :);
upAngle = @(a) acos((1 * a(3)) / norm(a));

%% Odometry (Angular Velocity)
odometryPosAngVel = zeros(l, 3);
odometryPosAngVel(1, :) = camPos(1, :); % inital position
oldPhi = phi(1);
zAngle = upAngle(initUpVector);

for i=2:l
    % With Angular Velocity:
    phi_dot = angVel(i, 3);
    
    % Get change in position 
    x_dot = v_combined(i) * cos(oldPhi - dt(i)*phi_dot); 
    y_dot = v_combined(i) * sin(oldPhi - dt(i)*phi_dot);  
    % Only if significant deviation from z-Axis = 1g
    if abs(acc(i, 3) - 1) > 0.02
        z_dot = v(i) * sin(zAngle - dt(i)*angVel(i, 2));
        zAngle = zAngle - dt(i)*angVel(i, 2);
    else 
        z_dot = 0;
    end
    oldPhi = (oldPhi - dt(i)*phi_dot); 
   
    % predict 
    odometryPosAngVel(i, 1) = odometryPosAngVel(i-1, 1) + x_dot * dt(i);% x-Pos
    odometryPosAngVel(i, 2) = odometryPosAngVel(i-1, 2) + y_dot * dt(i);% y-Pos
    odometryPosAngVel(i, 3) = odometryPosAngVel(i-1, 3) + z_dot * dt(i); % z-Pos
end

%% Odometry (Steering Angle)
odometryPosSteAng = zeros(l, 3);
odometryPosSteAng(1, :) = camPos(1, :); % inital position
oldPhi = phi(1);
zAngle = upAngle(initUpVector);

for i=2:l        
    % With Steering Angle: 
    x_dot = v_combined(i) * cos(oldPhi + dt(i)*steerAngle(i)); 
    y_dot = v_combined(i) * sin(oldPhi + dt(i)*steerAngle(i));  
    % Only if significant deviation from z-Axis = 1g
    if abs(acc(i, 3) - 1) > 0.02
        z_dot = v(i) * sin(zAngle + dt(i)*upAngle(acc(i, :)));
        zAngle = zAngle + dt(i)*upAngle(acc(i, :));
    else
        z_dot = 0;
    end
    oldPhi = (oldPhi + dt(i)*steerAngle(i));
    
    % predict 
    odometryPosSteAng(i, 1) = odometryPosSteAng(i-1, 1) + x_dot * dt(i);% x-Pos
    odometryPosSteAng(i, 2) = odometryPosSteAng(i-1, 2) + y_dot * dt(i);% y-Pos
    odometryPosSteAng(i, 3) = odometryPosSteAng(i-1, 3) + z_dot * dt(i); % z-Pos
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
    y_dot = v(i) * cos(eulerAngles(i, 3));
    x_dot = -v(i) * sin(eulerAngles(i, 3)); 
    z_dot = v(i) * sin(eulerAngles(i, 3));
    
    % predict 
    odometryPosCamera(i, 1) = odometryPosCamera(i-1, 1) + x_dot * dt(i);% x-Pos
    odometryPosCamera(i, 2) = odometryPosCamera(i-1, 2) + y_dot * dt(i);% y-Pos
    odometryPosCamera(i, 3) = odometryPosCamera(i-1, 3); % z-Pos
end

%% Predicted Position
predictedPos = zeros(l, 3);
predictedPos(1, :) = camPos(1, :); % inital position (the same as odometry position)%

for i=2:l
    deltaSteAng = odometryPosSteAng(i, :) - odometryPosSteAng(i - 1, :);
    deltaAngVel = odometryPosAngVel(i, :) - odometryPosAngVel(i - 1, :);
    deltaCamPos = camPos(i, :) - camPos(i - 1, :);
    predictedPos(i, :) = predictedPos(i-1, :) + (1-rateCamConf(camConf(i)))*(2*deltaAngVel + deltaSteAng)./3 + rateCamConf(camConf(i))*deltaCamPos;
end

%% Kalman Filter
f = @(k) [predictedPos(k, 1)+dt(k)*v_combined(k)*cos(phi(k));
          predictedPos(k, 2)+dt(k)*v_combined(k)*sin(phi(k));
          predictedPos(k, 3)+dt(k)*v_combined(k)*sin(theta);
          phi(k)+dt(k)*phi_dot(k);
          theta(k)+dt(k)*theta_dot(k)];

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
%scatter(odometryPosCamera(:, 1), odometryPosCamera(:, 2), 'm')
%hold on
%scatter(odometryPosCompass(:, 1), odometryPosCompass(:, 2), 'r')
%hold on
scatter(odometryPosAngVel(:, 1), odometryPosAngVel(:, 2), 'c')
hold on
scatter(odometryPosSteAng(:, 1), odometryPosSteAng(:, 2), 'g')
hold on
scatter(predictedPos(:, 1), predictedPos(:, 2), 'y')
title('Cam vs Odometry in 2D')
