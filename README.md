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
- Some issues with model, unclear what cause
- No Speed data was recorded last time (check cables and arduino connection)
- Check delta again
    - With Interrupt data is send only 12times/sec
    - Without between 18 and 23 times

TODO:
- Maybe seperate code in different modules
- Create mathematical model of robot
    - Implement Kalman Filter, propably extended
