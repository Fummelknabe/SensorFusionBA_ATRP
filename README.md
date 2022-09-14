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
    - l_f = 18cm
    - l_r = 50cm
- Correspondance between speedValue and m/s:
    - speedValue = round(19 + v*2.1) with 36 km/h top speed
- Battery Width: 15.5cm

BA-NOTES:
- Mention: Design of Software, Degrees of Freedom, Ackerman

TODO:
- Maybe:
    - Redo Camera Movement
    - Detect Wheel Slippage (two solutions plausible:
        1. If camera position change is under 90° but orientation is way smaller
        2. If camera position change is under 90° but angular vel is small)
- Close appropriate windows on disconnect
