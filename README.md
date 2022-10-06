# 2022 Kevin Robot  
# The software is located at https://thedirtymechanics.com/bitbucket/projects/ROB22/repos/2022-kevin/browse

# Operator - Game Pad - In pad mild or agreesive mode
* Right Bumpter -- Drive Stright
* Left Bumper - Shoot - (Turns on the shooter motor and pushes Frisbee into the shooter)
* POV up - shooter off,  right - shooter slow, down - shooter medium speed, left - shooter high speed
* Right Joy - controls right motors
* Left Joy - controls left motors

# Operator - Gaame Pad -- Arcade Mode
* In arcade mode, the joystick y axis controls the forward and reverse motion of the robot. You will need to set both motors to the power level indicated by the y axis deflection. That controls forward and backward motion. So how do we turn? We use the x axis (side to side) deflection to cause the motors to run at different speeds by adding the x value to one motor power level and subtracting the x value from the other motor power level

# Motors ID's
* 2 Right Drive
* 3 Left Drive
* 4 Right Follow
* 5 Left Follow
* 12 Shooter 

# Pneumatic ID's
* 0,1 Frisbee push cylinder -- it pushes the Frisbee into the shoot motor
* 2,3 Frisbee drop cylinder -- activating this cylinder causes the Fribee to drop into position for shooting

# HW Things to complete
* Mount and configure all modules

# SW Items needed
* Add vision logic for April tags
* Test shoot logic with real robot
* Test arcade drive
* Add blinking lamp on PCM
* Get rumble working
* Finish up the motor interface for the other motor types
