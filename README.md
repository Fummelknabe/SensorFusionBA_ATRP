# SensorFusionBA_ATRP
Sensor Fusion Design and Evaluation for Robot Motion Estimation in Uneven Terrain

VEHICLE STATS:
- Axis Difference: 71cm
- Wheel Distance: 45cm
- Camera Offset: 5cm
- Camera Height: 47cm
- IMU Height: 39cm
- IMU Offset: 10cm

ISSUES:
- Plot window should be scalable
- Recentering of Plots (give same view as restart)
- After Disconnecting, no reconnecting possible
    -> Restart required 

TODO:
- Attach IMU
- Display recorded data:
    - > Plot Position in positional graph
    - > Display raw data
- Maybe seperate code in different modules
- Create mathematical model of robot
    - > Implement Kalman Filter, propably extended
