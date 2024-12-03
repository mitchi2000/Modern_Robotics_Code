import numpy as np 
import csv
import modern_robotics as mr
import matplotlib.pyplot as plt

# functions to calculate trajectory

def segments(X1,X2,X3,k,timeSteps,gripper_state,Tf):
    Xstart=X1  # start configuration 
    if X3 is not None:
        Xend = X2 @ X3  # end configuration if X3 is provided
    else:
        Xend = X2 
    N = int((Tf * k) / timeSteps) # number of trajectory points
    segTrajectory=mr.ScrewTrajectory(Xstart,Xend,Tf,N,method=3)
    

    trajectory_with_gripper_state = [] # add each transformation matrix with the gripper state
    for matrix in segTrajectory:
        trajectory_with_gripper_state.append((matrix, gripper_state))
    
    return trajectory_with_gripper_state,Xend

def change_gripper_state(grasp_pos,TimeNeeded,TimeSteps,gripper_state):

    keep_still=[]
    num_steps = int(TimeNeeded / TimeSteps)
    for i in range(num_steps):
        keep_still.append((grasp_pos, gripper_state))

    return keep_still


# functions to calculate Next step

def SpeedLimiting(controls, wheelSpeed, armSpeed):
    # Speed limit for wheels
    wheels = controls[:4]
    for i in range(len(wheels)):
        if wheels[i] > wheelSpeed:
            controls[i] = wheelSpeed  # Update the control value in the list
        elif wheels[i] < -wheelSpeed:
            controls[i] = -wheelSpeed  # Update the control value in the list

    # Speed limit for arm joints
    armJoints = controls[4:9]
    for i in range(len(armJoints)):
        if armJoints[i] > armSpeed:
            controls[i+4] = armSpeed  # Update the control value in the list
        elif armJoints[i] < -armSpeed:
            controls[i+4] = -armSpeed  # Update the control value in the list

    return controls

def calculateDelta_q(Vb,phi_k):
    wbz=Vb[0] # this is the angular velocity around Z axis 
    vbx=Vb[1] # this is the linear velocity along X axis 
    vby=Vb[2] # this is the linear velocity along Y axis 

    # calculate ∆qb in {b} frame
    if wbz==0:
        delta_qb = np.array([0, vbx, vby])
    else:
        delta_qb = np.array([
            wbz,
            (vbx * np.sin(wbz) + vby * (np.cos(wbz) - 1)) / wbz,
            (vby * np.sin(wbz) + vbx * (1 - np.cos(wbz))) / wbz
        ])

    # calculate ∆q in {s} frame
    T= np.array([
    [1, 0, 0],
    [0, np.cos(phi_k), -np.sin(phi_k)],
    [0, np.sin(phi_k), np.cos(phi_k)]    ])

    delta_q=T @ delta_qb

    return delta_q

def save_to_csv(data, filename):

    try:
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            
            # Write each row of the data into the file
            for row in data:
                writer.writerow(row)
        
        print(f"Configuration has been successfully saved to '{filename}'")
    except Exception as e:
        print(f"An error occurred while saving the file: {e}")

def NextStep(controls,config,deltat,gripper_state):
    # Define the variables
    r = 0.0475  # Wheel radius in meters
    l = 0.47 / 2  # Forward-backward distance (half) in meters
    w = 0.3 / 2   # Side-to-side distance (half) in meters

    # Define the F matrix
    F = (r / 4) * np.array([
        [-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
        [1, 1, 1, 1],
        [-1, 1, -1, 1]
    ])
    # calculate Vb
    wheelSpeed=np.array(controls[:4]) # extract wheel speed from the 9 vector
    deltaThetaWheel=wheelSpeed*deltat # calculate delta theta
    Vb = F @ deltaThetaWheel   #we calculate Vb using : Vb=F∆o
    # print(Vb)

    # calculate ∆q in {s}frame
    phi_k=config[0]
    delta_q=calculateDelta_q(Vb,phi_k)
    # print(f"your new ∆q is {delta_q}")


    #calculate wheel angles
    wheelAngles=np.array(config[8:12]) #get the wheel angles
    newWheelAngles=wheelAngles+deltaThetaWheel # calulta the new wheel angles 
    # print(f" new wheel angles is {newWheelAngles}")

    # Calculate arm joint angles
    jointAngles = np.array(config[3:8])  # extract the current joint angles 
    jointSpeeds = np.array(controls[4:9])  # extract the arm joint speeds 
    deltaThetaJoints = jointSpeeds * deltat   # calculate delta theta for arm joints
    newJointAngles = jointAngles + deltaThetaJoints  # calculate new arm joint angles
    # print(f" New Arm Joints angles is {newJointAngles}")


    # after the calculation update the 13 vectors to match the changes in ∆q and new wheel and joint angles
    config[0] += delta_q[0] # Update chassis_phi
    config[1] += delta_q[1] # Update chassis_x 
    config[2] += delta_q[2] # Update chassis_y 
    config[8:12] = newWheelAngles
    config[3:8] = newJointAngles
    config[12]=gripper_state

    return config

# feedback control functions


def calculate_Tse(config):
    
    phi=config[0]  # extract chassis phi
    x=config[1]        # x position of chassis
    y=config[2]        # y position of chassis
    theta=config[3:8]      # arm joints angles (j1--->j5)
    
    # get frame{b} position relative to {s} from q
    T_sb = np.array([  
        [np.cos(phi), -np.sin(phi), 0, x],
        [np.sin(phi), np.cos(phi), 0, y],
        [0, 0, 1, 0.0963],
        [0, 0, 0, 1]        
    ])
    #{0} base of the arm relative to frame {b} 
    T_b0 = np.array([
        [1, 0, 0, 0.1662],
        [0, 1, 0, 0],
        [0, 0, 1, 0.0026],
        [0, 0, 0, 1]
    ])
    # home position of arm relative to it's base
    M_0e = np.array([
        [1, 0, 0, 0.033],
        [0, 1, 0, 0],
        [0, 0, 1, 0.6546],
        [0, 0, 0, 1]
    ])
    # the screw axis of each joint represented as columns B=(w,v)
    B_list = np.array([
            [0,       0,       0,       0,       0],
            [0,      -1,      -1,      -1,       0],
            [1,       0,       0,       0,       1],
            [0,  -0.5076, -0.3526, -0.2176,       0],
            [0.033,   0,       0,       0,       0],
            [0,       0,       0,       0,       0]
        ])

    T_0e=mr.FKinBody(M_0e,B_list,theta)

    Tbe=T_b0 @ T_0e  # calculate Tb0 T0e(o) first
    Tse=T_sb @ Tbe    # calculate Tse 

    return Tse

def calculate_Je(configuration):

    B_list = np.array([
            [0,       0,       0,       0,       0],
            [0,      -1,      -1,      -1,       0],
            [1,       0,       0,       0,       1],
            [0,  -0.5076, -0.3526, -0.2176,       0],
            [0.033,   0,       0,       0,       0],
            [0,       0,       0,       0,       0]
        ])

    theta_list=configuration[3:8]
    J_arm = mr.JacobianBody(B_list, theta_list)
    # Define the variables
    r = 0.0475  # Wheel radius in meters
    l = 0.47 / 2  # Forward-backward distance (half) in meters
    w = 0.3 / 2   # Side-to-side distance (half) in meters

    # Define the F matrix
    F = (r / 4) * np.array([
        [-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
        [1, 1, 1, 1],
        [-1, 1, -1, 1]
    ])

    T_b0 = np.array([
        [1, 0, 0, 0.1662],
        [0, 1, 0, 0],
        [0, 0, 1, 0.0026],
        [0, 0, 0, 1]
    ])
    M_0e = np.array([
        [1, 0, 0, 0.033],
        [0, 1, 0, 0],
        [0, 0, 1, 0.6546],
        [0, 0, 0, 1]
    ])

    T_e0=mr.TransInv(mr.FKinBody(M_0e,B_list,theta_list)) #T_e0
    T_0b=mr.TransInv(T_b0)
    Adj_Teb=mr.Adjoint(T_e0 @ T_0b)

    m = F.shape[1]  # get the propper F6 format
    F_6 = np.zeros((6, m))   
    F_6[2:5, :] = F 

    J_base = Adj_Teb @ F_6  # get the J_base

    J_e = np.hstack((J_base, J_arm))

    return J_e

def Wheel_And_Joint_Speeds(Ve,Je):
    
    Je_pseudo_inv = np.linalg.pinv(Je)
    
    # Calculate the wheel speeds and joint speeds
    speeds = np.dot(Je_pseudo_inv, Ve)

    return speeds

def FeedbackControl(Xd,Xd_Next,configuration, Kp, Ki, integral,timeStep):
    X=calculate_Tse(configuration) # get X
    
    Xerr=mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X), Xd)))
    integral  += Xerr * timeStep  # Increment the running total
    T_relative=mr.TransInv(Xd) @ Xd_Next  # Xd^-1 @ Xd,Next
    Vd = mr.MatrixLog6(T_relative)  # calculate Vd
    
    Vd = (1 / timeStep) * mr.se3ToVec(Vd) # multiply by 1/∆t

    Ad_Xinv_Xd = mr.Adjoint(mr.TransInv(X) @ Xd)  # Adjoint(X^-1 @ Xd)
    Vd = Ad_Xinv_Xd @ Vd  # Adjusted feedforward twist

    Ve = Vd + (Kp @ Xerr) + (Ki @ integral)  # calculate ve (error twist)
    

    return Ve, Xerr ,integral

# plotting function for Xerr

def plot_Xerr(all_X_err):
    fig, ax = plt.subplots()
    ax.plot(all_X_err)  # Matplotlib will automatically plot each component of the error as separate lines
    ax.set_title("Evolution of Xerr Over Time")
    ax.set_xlabel("Time Steps")
    ax.set_ylabel("Error Components")
    ax.grid(True)
    plt.show()




# Initial configuration of the robot

configuration = [
    1.3,    # chassis_phi #1.3
    -0.5,    # chassis_x #-0.5
    0.8,    # chassis_y   #1
    0,    # J1 (first arm joint)
    0,    # J2 (second arm joint)
    -0.5,    # J3 (third arm joint)  #-0.5
    -0.6,    # J4 (fourth arm joint) #-1.1
    0,    # J5 (fifth arm joint)
    0,    # W1 (first wheel)
    0,    # W2 (second wheel)
    0,    # W3 (third wheel)
    0,     # W4 (fourth wheel)
    0     # gripper state
]


# trajectory generation  inputs and function call

standoff_Zpos=+0.15 # 10 cm


Tse_initial=np.array([  # e.e initial configuration
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [-1, 0, 0, 0.5],
    [0, 0, 0, 1]    ])

# cube position for ################# best ######################
                    #############################################

Tsc_initial = np.array([  # cube initial configuration
    [1, 0, 0, 1],
    [0, 1, 0, 0],
    [0, 0, 1, 0.025],
    [0, 0, 0, 1]    ])

Tsc_goal = np.array([    # cube goal configuration
    [0, 1, 0, 0],
    [-1, 0, 0, -1],
    [0, 0, 1, 0.025],
    [0, 0, 0, 1]   ])

# cube position for  ################# newTask ######################
                    #############################################
# Tsc_initial = np.array([  # cube initial configuration
#     [0, -1, 0, 1],
#     [1, 0, 0, 1],
#     [0, 0, 1, 0.025],
#     [0, 0, 0, 1]    ])

# Tsc_goal = np.array([    # cube goal configuration
#     [1, 0, 0, 2],
#     [0, 1, 0, 0],
#     [0, 0, 1, 0.025],
#     [0, 0, 0, 1]   ])




Tce_grasp = np.array([ #grasp configuration of {e} relative to {c} 
    [0, 0, 1, 0.008],
    [0, 1, 0, 0],
    [-1, 0, 0, 0],
    [0, 0, 0, 1]])

Tce_standoff = np.array([ # standoff position 1 of the e.e 10 cm above the cube 
    [0, 0, 1, 0.008],
    [0, 1, 0, 0],
    [-1, 0, 0, 0.025+standoff_Zpos],
    [0, 0, 0, 1]])


# calculate the full trajectory needed for the end effector to make

Fulltrajectory=[]  # create a list to append all the needed segments

Tf1=6 # adjust how much time each segment needs to complete task (in seconds)
traj1,Xend1=segments(Tse_initial,Tsc_initial,Tce_standoff,k=1,timeSteps=0.01,gripper_state=0,Tf=Tf1) # segment 1
Fulltrajectory+=traj1 #from start config to standoff 1 config

Tf2=1 
traj2,Xend2=segments(Xend1,Tsc_initial,Tce_grasp,k=1,timeSteps=0.01,gripper_state=0,Tf=Tf2) # segment 2
Fulltrajectory+=traj2 # from standoff 1 config to grasp config

Tf3=0.8
segment3=change_gripper_state(Xend2,TimeNeeded=Tf3,TimeSteps=0.01,gripper_state=1) # segment 3
Fulltrajectory+=segment3 # close gripper

Tf4=1
traj3,Xend3=segments(Xend2,Xend1,None,k=1,timeSteps=0.01,gripper_state=1,Tf=Tf4) # segment4
Fulltrajectory+=traj3 # go back 2 standoff position after cube is caught

Tf5=4.5
traj4,Xend4=segments(Xend3,Tsc_goal,Tce_standoff,k=1,timeSteps=0.01,gripper_state=1,Tf=Tf5) # segment 5
Fulltrajectory+=traj4 # go from standoff 1 to standoff 2 while griper closed

Tf6=1
traj5,Xend5=segments(Xend4,Tsc_goal,Tce_grasp,k=1,timeSteps=0.01,gripper_state=1,Tf=Tf6) #segment 6
Fulltrajectory+=traj5 # go from standoff 2 to goal position

Tf7=0.8
segment7=change_gripper_state(Xend5,TimeNeeded=Tf7,TimeSteps=0.01,gripper_state=0) # segment 7
Fulltrajectory+=segment7 # open gripper at goal config

Tf8=1
traj6,Xend6=segments(Xend5,Xend4,None,k=1,timeSteps=0.01,gripper_state=0,Tf=Tf8) #segment 8
Fulltrajectory+=traj6 # go back to standoff postion 2 

#  Feedback control for "best" and "newTask"
K_p = np.eye(6) * 2 
K_i = np.eye(6) * 0.001 

# Feedback control for "overshoot"
# K_p = np.eye(6) * 2.25 
# K_i = np.eye(6) * 1.75
 
integral_error = np.zeros((6,),dtype=float)  # initialize integral error 




timeStep=0.01
WheelSpeedLimit=20 # you can choose diffrent speed limits for arm and wheels 
ArmSpeedLimit=20
 

simulation_time=Tf1+Tf2+Tf3+Tf4+Tf5+Tf6+Tf7+Tf8 # choose how long do you want the simulation to last in aseconds (add all the Tf forsegment generation )
numLoops=int(simulation_time/timeStep) # calculate to get hte numbers of loops you need to simulate
simulationSteps =[]   # store initial configuration as the first step
all_Xerr=[]
simulationSteps.append(configuration.copy())  # the 13 vector trajectory 

for i in range(numLoops-1):

    # calculated the controls needed to generate the controls needed to upgrade the configuration 
    Ve,Xerr,integer=FeedbackControl(Fulltrajectory[i][0],Fulltrajectory[i+1][0],configuration,K_p,K_i,integral_error,timeStep)
    integer=integral_error
    all_Xerr.append(Xerr.copy())  # add each Xerr configuration into an array
    Je=calculate_Je(configuration)   # calculate the robots Jacobian
    controls=Wheel_And_Joint_Speeds(Ve,Je) # use the robot jacobian and commanded twist to calculate the controls needed
    controls=SpeedLimiting(controls,WheelSpeedLimit,ArmSpeedLimit) # addjust the speed based on your needs 

    # calculate the new configuration based on these new controls recieved
    configuration = NextStep(controls, simulationSteps[i],timeStep,Fulltrajectory[i][1])  # pass the latest configuration
    simulationSteps.append(configuration.copy())  # add the new configuration to the list


save_to_csv(simulationSteps,filename="youBot_configuration.csv") # save each configuration at each timestep into a csv file
save_to_csv(all_Xerr,filename="Xerr.csv")   # save Xerr at each loop into a .csv file
plot_Xerr(all_Xerr)    # plot the resulting Xerr over time
