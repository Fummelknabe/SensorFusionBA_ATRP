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
- Correspondance between speedValue and m/s:
    - speedValue = round(19 + v * 1.325) (only a rough estimate)
    - That is not realistic as it would settle  for 57 km/h top speed
    - Better estimate: round(19 + v*2.1) with 36 km/h top speed
- Battery Width: 15.5cm

ISSUES:
- Acceleration data is not useful other to determine up direction
    - Maybe Quad is accerlerating too fast and it is often not sensed by imu
    - Only when v_k-1 != v_k != v_k+1 then acceleration can be used
- No sensed steering angle was recorded 

TODO:
- Maybe seperate code in different modules
- Create mathematical model of robot
    - Implement Kalman Filter, propably extended
- Redo Camera Movement
- Fix Model movement
- Redo Sensorfusion
