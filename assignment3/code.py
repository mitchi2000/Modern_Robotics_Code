# Import the necessary libraries

import numpy as np
import modern_robotics as mr
import csv


M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]

g = np.array([0, 0, -9.81])  # gravity vector in the -z direction
intRes = 8     #  number of integration steps to take during each timestep
dthetalist = np.zeros(6) #  joint velocities (zero)
dt = 0.01  # Time step (at least 100 steps per second)


# scenario 1 

thetalist1 = np.zeros(6)  #  joint angles (home configuration)
tf1 = 3  # Total time of simulation (3 seconds)
N1 = int(tf1 / dt)  # Number of time steps 
taumat1 = np.zeros((N1, 6)) # creat the tau matrix (filled with zeros)
Ftipmat1 = np.zeros((N1, 6)) # External forces and torques (zero)


[thetamat1,dthetamat1] =mr.ForwardDynamicsTrajectory(thetalist1,dthetalist,taumat1,g,Ftipmat1,Mlist,Glist,Slist,dt,intRes)

# Define the file name
csv_filename1 = "simulation1.csv"
# Write the joint angles to the CSV file
with open(csv_filename1, mode='w', newline='') as file:
    writer = csv.writer(file)
    for row in thetamat1:
        writer.writerow(row)

print(f"Joint angles have been saved to {csv_filename1}")




# scenario 2

thetalist2 = np.array([0, -1, 0, 0, 0, 0])  # Initial joint angles
tf2 = 5  # Total time of simulation (3 seconds)
N2 = int(tf2 / dt)  # Number of time steps 
taumat2 = np.zeros((N2, 6)) # creat the tau matrix (filled with zeros)
Ftipmat2 = np.zeros((N2, 6)) # External forces and torques (zero)


[thetamat2,dthetamat2] =mr.ForwardDynamicsTrajectory(thetalist2,dthetalist,taumat2,g,Ftipmat2,Mlist,Glist,Slist,dt,intRes)

# Define the file name
csv_filename2 = "simulation2.csv"
# Write the joint angles to the CSV file
with open(csv_filename2, mode='w', newline='') as file:
    writer = csv.writer(file)
    for row in thetamat2:
        writer.writerow(row)

print(f"Joint angles have been saved to {csv_filename2}")