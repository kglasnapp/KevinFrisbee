
# 2022 Sibling Robot  
# The software is located at https://thedirtymechanics.com/bitbucket/projects/ROB22/repos/2022-sibling/browse 

# Operator - Game Pad
* A 1 - Intake - out (Drop intake, turn on intake motor, turn on lower motor) 
* B 2 - Intake - in (Raise intake, turn off intake motor, turn off lower motor)

* Right Bumper 6 - Default Shot
* Start 8 - Shooter On - (Turn on shooter motor, Enable vision tracking) -- Shooter and vision will be active when robot starts, needed only for testing
* Back 7 - Shooter off - (Turn off shooter motor, Disable vision tracking) -- Shooter and vision will be active when robot starts, needed only for testing
* Left Bumper 5 - Shoot - (Turns on the top magazine motor, shooter motor will always be on)
* POV long shot hit POV-up, short shot hit POV-down, shot OK hit POV right or left data this daata is logged
* Hood Button -- Clear power pannel sticky faults
    
# Driver - Game Pad
* Right Joy 5 - Speed of right drive motors
* Left Joy 1 - Speed of left drive motors
* Left Bumper 5 -  Drive Straight
* Y 4 - Zero Yaw
* POV - control the speed of shooter testing only

# Automatic tasks
* Move the shooter turret to center the target using the x coordinate
* If target not visible turn opposite direction, keep spinning until target visible
* Speed of shooter andd angle of the hood is determined by the y coordinate of the target

# Motors ID's
* 2,4 Right Drive and follow
* 3,5 Left Drive and follow
* 9 Hood
* 12 Shooter 
* 11 Turret
* 14 Backspin valid when backSpinID is greater than 0
* 15 Beater Bar
* 18 topMagazineID
* 19 bottomMagazineID
* 20 Climber

# Pneumatic ID's
* 0,1,2,3 for the High and Low pressure cylinders
* 6,7 for the latching pneumatic

# Get Current and Voltage
* `PDH_CAN_ID` in _src/main/java/frc/robot/Robot.java_ should match your device's CAN ID

### Steps:
* 1. Deploy the code to your roboRIO with the PDH connected.
* 2. Open the included _shuffleboard.json_ from this project in Shuffleboard.
* 3. Channel currents, total current, and input voltage should be displayed on Shuffleboard.

# Things that need to be accomplished before the robot is ready for Vierra. 

# HW Things to complete
* a) Connect the pneumatic to the climber - priority
* b) Add side pannel for wire protection - priority
* c) Add ribbon cable covers for all SRX - priority
* d) Add wire dressing to the hood wires - priority
* e) Add button to power off the Limelight - priority
* f) Continue to clean up the wiring and pneumatic hoses
* g) Add sensors for balls in lower magazine, mid magazine and upper magazine.
* h) Raise Power Hub off belly pan to allow for better access
* i) Label wires for improved debugging


# SW Items needed
* a) Test improved drive modes mild - agreesive joy - operator.
* b) Improve the speed of the vision tracking so we can shoot and drive at the same time
* c) Addd the logic for the autonomous cases - priority
* d) Continue to dial in the shooter / hood
* e) Add buttons too do a short shot if no target, long shot if no target
* f) limelight sudo raspi-config option 6 in first menu

* Why this log when disabled
* 23:04:12-139 Turret State:TARGETING Vision valid:true x:-20.96 Angle:982.36 Limit <R:false,L:false>
* 23:04:12-159 Turret State:TARGETING Vision valid:true x:-20.96 Angle:982.36 Limit <R:false,L:false>
* 23:04:12-178 Turret State:TARGETING Vision valid:true x:-20.96 Angle:982.36 Limit <R:false,L:false>

* 13:55:59-683 Odometry Shoot suggested angle:-90.0 x:45.0 y:0.0 robot angle:0.0
* 13:55:59-705 Odometry Shoot suggested angle:90.0 x:-45.0 y:0.0 robot angle:0.0
* 13:55:59-722 Odometry Shoot suggested angle:0.0 x:0.0 y:-45.0 robot angle:0.0
* 13:55:59-742 Odometry Shoot suggested angle:-135.0 x:45.0 y:45.0 robot angle:0.0
* 13:55:59-762 Odometry Shoot suggested angle:-115.0 x:45.0 y:45.0 robot angle:-20.0
* 13:55:59-782 Odometry Shoot suggested angle:115.0 x:-45.0 y:45.0 robot angle:20.0


* Hot glue RSl both robots
* Fix pneumatic for ball
* Fix climber fix 8020
* Tune the sibling shooter
* Take off camera power switch
* 120 inches power need increase
* Take off connectors black tape
* Limit switch for hood broke


