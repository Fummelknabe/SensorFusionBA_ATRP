# SensorFusionBA_ATRP
Sensor Fusion Design and Evaluation for Robot Motion Estimation in Uneven Terrain

VEHICLE STATS:
- Mass: 43.0kg
- Axis Difference: 69cm
- Wheel Distance: 45cm
- Camera Offset: 5cm
- Camera Height: 47cm
- IMU Height: 39cm
- IMU Offset: 10cm
- IMU is vehilce reference point 
    - l_f = 59cm
    - l_r = 10cm

ISSUES:
- Some issues with model, unclear what cause
- Transform cam pos x-z value to 2d plane with imu accelerometer
- Record delta time in positional data
- Acceleration data is not useful other to determine up direction
    - Quad is accerlerating too fast and it is often not sensed by imu
    - Only when v_k-1 != v_k != v_k+1 then acceleration can be used
- IMU data too noisy -> maybe track more datapoints or install better suspensions

TODO:
- Maybe seperate code in different modules
- Create mathematical model of robot
    - Implement Kalman Filter, propably extended
- Reduce update time of speed and steer angle sensor
- Fix depiction of prediction + camera (maybe try and disable camera)
