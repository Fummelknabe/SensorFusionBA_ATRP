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
- Acceleration data is not useful other to determine up direction
    - Maybe Quad is accerlerating too fast and it is often not sensed by imu
    - Only when v_k-1 != v_k != v_k+1 then acceleration can be used
- Spikes in delta time for max 1.4 seconds
    - At Peaks, Peaks also in angular Velocity around z Axis
- Magnetic Field values of imu not working correctly
- No steering angle was recorded the last data samples

TODO:
- Maybe seperate code in different modules
- Create mathematical model of robot
    - Implement Kalman Filter, propably extended
- Fix depiction of prediction + camera 
