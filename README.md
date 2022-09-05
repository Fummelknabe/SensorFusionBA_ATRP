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
    - speedValue = round(19 + v*2.1) with 36 km/h top speed
- Battery Width: 15.5cm

TODO:
- Maybe:
    - Seperate code in different modules
    - Redo Camera Movement
- Fix Model movement
- Redo Sensorfusion
    - Look at literature
- Fix backward motion

## Arbitrary Information
Installing cfclient on windows:
    pip3 install cfclient (if python 3.7 > is avaiable)
For crazyradio on windows:
    https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/usbwindows/
    - Installing of driver is necessary 