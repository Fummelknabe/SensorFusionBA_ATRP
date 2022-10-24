# SensorFusion for vehicle in uneven terrain

## Execute software
To use this software, Julia V1.7 or newer is required. 

Start the program from REPL: 
1. Start Julia REPL in folder containing the repository
2. Go into package manager by pressing `]`
3. Activate project environment `activate .`
4. Download and compile dependencies `instantiate`
5. Leave package manager
6. Run `julia --project src/Main.jl`

## How to use the software
The menu bar on the top of the GUI is the main interaction point with the software. Here the __Help__ button
opens a small window to show basic controls of _Dear ImGui_ and for controlling the robot. 

The __Load Data__ button lets you load raw positional data to analyze. A _.json_ file has to be choosen that contains the
correct data to load. After loading an appropriate file a new window opens that display the data. The amount of data points can be adjusted by the use of the slider. The pose estimation and with that the sensorfusion will be enabled if
the button __Update Estimation__ is toggled. The parameters of the estimation can be adjusted by pressing the __Open Settings__
button. __Load True Pos Data__ lets you load another _.json_ file containing the true information data which will be overlayed
in the display. 

The connect window lets you connect to the Jetson Nano by entering its port and IP address. If you enter no specific one
the default one will be used, which is used in the Python program controlling the robot.

Once you are connected new menus appear in the menu bar. The __Render Robot__ button toggles the rendering of the robot in
its current orientation. Also there will be a button __Record Data__ which lets you upon entering the amount of data points record data. The recorded data is saved in a _.json_ file in the data folder of the project. This file can then be loaded and
analyzed. 
There is also the possibility to send automatic commands to the robot. The dialog for that lets you chain commands together
which will be send to the robot when confirming.
