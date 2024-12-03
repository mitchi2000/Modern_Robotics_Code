This project implements a simulation of the youBot robot performing a series of kinematic tasks, such as trajectory planning and following, while adhering to the constraints of the robot's dynamics. Key functionalities include:

# Generating a trajectory for the robot's end-effector to move from an initial to a goal configuration while interacting with objects.
# Computing control inputs to execute the trajectory.
# Implementing feedback control to minimize positional errors during movement.
# Saving and visualizing data to analyze results.

this projecte uses "ScrewTrajectory" from the modern robotics library to generate an 8 segment trajectory for the
e.e to follow the resulting trajectory is then used to calculate the wheel and joint speed needed to drive the robot to it's 
destenation The feedback control ensures that the end-effector tracks the desired trajectory by correcting for errors at each timestep
that is done in the "feedbackControl" function then the wheel and joint speeds are fed to a function called NextStep that calculates
the robots new configuration based on these controls

there's 3 variations of the code the example inputs for these 3 are commented in the code.py file 
# one is "best" with k_p=2 and k_i=0.001 with  Cube pos : Initial:(x,y,theta)=(1,0,0) and Goal:(x,y,theta)=(0,-1,-pi/2)

# one is "overshoot" with k_p=2.25 and k_i=1.75  with the same Cube position

# and the third one is "newTask" with the K_p and K_i the same as "best" but with diffrent Cube pos : Initial:(x,y,theta)=(1,1,pi/2) and Goal:(x,y,theta)=(2,0,0)

all libraries needed for this are :
#  numpy  
#  csv
#  modern_robotics 
# matplotlib.pyplot

