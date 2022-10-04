
# 2022 Sibling Robot  
# The software is located at https://thedirtymechanics.com/bitbucket/projects/ROB22/repos/2022-sibling/browse 
# Operator - Game Pad
* A 1 - Intake - out (Drop intake, turn on intake motor, turn on lower motor) 
* B 2 - Intake - in (Raise intake, turn off intake motor, turn off lower motor)
* X 3 - Prepare Climb - Raise Climber to allow driver to connect with bar, unlock climber solenoid
* Right Joy 5 - Raise or Lower Climber (If possible lock the climber solenoid after 1 one second of no movement need to test this)
* Y 4 - Climber Lock - Lock the climber solenoid 
* Right Bumper 6 - Reverse intake
* Start 8 - Shooter On - (Turn on shooter motor, Enable vision tracking) -- Shooter and vision will be active when robot starts, needed only for testing
* Back 7 - Shooter off - (Turn off shooter motor, Disable vision tracking) -- Shooter and vision will be active when robot starts, needed only for testing
* Left Bumper 5 - Shoot - (Turns on the top magazine motor, shooter motor will always be on)

# Driver - Game Pad
* Right Joy 5 - Speed of right drive motors
* Left Joy 1 - Speed of left drive motors
* Left Bumper 5 -  Drive Straight
* Y 4 - Zero Yaw
* POV - control the speed of shooter testing only

# Automatic tasks
* Move the shooter turret to center the target using the x coordinate
* If target not visible turn opposite direction, keep spinning until target visible
* Speed of shooter is determined by the y coordinate of the target, when hood is working use y coordinate to determine shooter speed and angle of hood

# Motors ID's
* 2,4 Right Drive and follow
* 3,5 Left Drive and follow
* 12 Shooter 
* 11 Turret
* 14 Backspin valid when backSpinID is greater than 0
* 15 Beater Bar
* 18 topMagazineID
* 19 bottomMagazineID
* 20 Climber

# Pneumatic ID's
* 0,1,2,3 for the High and Low pressure cylinders

# Get Current and Voltage
* `PDH_CAN_ID` in _src/main/java/frc/robot/Robot.java_ should match your device's CAN ID

### Steps:
*1. Deploy the code to your roboRIO with the PDH connected.
*2. Open the included _shuffleboard.json_ from this project in Shuffleboard.
*3. Channel currents, total current, and input voltage should be displayed on Shuffleboard.

# Things that need to be accomplished before the robot is ready for Vierra. 


# HW Things to complete
a) Mount and test the new climber
b) Complete the design for the new shooter with the deflector
c) Continue to clean up the wiring and pneumatic hoses
d) Convert the side panels from wood to acrylic????
e) Add sensors for balls in lower magazine, mid magazine and upper magazine.
f) Add the RSL light

# SW Items needed
a) Test improved ramping function to allow driving with the game pad. Revert to joysticks if we can't get the pad working for driving  -- Tunned for Keanu and then for Riley -- need some more tuning
b) Test velocity PID to better control the speed of shooter wheels as balls are shot. 
c) Add logic to control the speed of the shooter based upon the limelight Y angle.
d) Improve the speed of the vision tracking so we can shoot and drive at the same time
e) Add logic to put in the log data that shows if the shot was successful. Use operator POV shot long hit up, short short hit down, Shot OK hit right or left
f) Addd the logic for the autonomous cases
g) Add code for the climber -- Joy stick to control up and down, button to release latch
h) Add option to vary drive parms
