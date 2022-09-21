# SensorFusion for vehicle in uneven terrain

## Execute software
To use this software, Julia V1.7 or newer is required. 

Start the program from REPL: 
1. Start Julia REPL in folder containing the repository
2. Go into package manager by pressing ']'
3. Activate project environment 'activate .'
4. Download and compile dependencies 'instantiate'
5. Leave package manager
6. Run 'julia --project src/Main'

## How to use the software
The menu bar on the top of the GUI is the main interaction point with the software. Here the 'Help' button
opens a small window to show basic controls of 'Dear ImGui' and for controlling the robot. 

The 'Load Data' button lets you load raw positional data to analyze. A .json file has to be choosen that contains the
correct data to load. After loading an appropriate file a new window opens that display the data. The amount of data points can be adjusted by the use of the slider. The pose estimation and with that the sensorfusion will be enabled if
the button 'Update Estimation' is toggled. The parameters of the estimation can be adjusten by pressing the 'Open Settings'
button. 'Load True Pos Data' let's you load another .json file containing the true information data which will be overlayed
in the display. 

The connect window let's you connect to the Jetson Nano by entering its port and IP address. If you enter no specific one
the default one will be used, which is used in the Python program controlling the robot.

Once you are connected new menus appear in the menu bar. The 'Render Robot' button toggles the rendering of the robot in
its current orientation. Also there will be a button 'Record Data' which lets you upon entering the amount of data points record data. The recorded data is saved in a .json file in the data folder of the project. This file can then be loaded and
analyzed. 
There is also the possibility to send automatic commands to the robot. The dialog for that lets you chain commands together
which will be send to the robot when confirming.


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
- What about the mag to influence state? I should rework that