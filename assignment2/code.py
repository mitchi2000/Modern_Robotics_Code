
import numpy as np
import modern_robotics as mr  # Assuming you have imported the necessary library for FKinBody and JacobianBody functions

def IKinBodyIterates(Blist, M, Tsd, theta0, eOmg, ev):
    thetalist = np.array(theta0).copy()
    i = 0
    maxiterations = 20

    # Initialize joint vectors storage
    joint_vectors = []

    # Initialize iteration table
    iterationTable = []

    # Initial end-effector configuration
    Tsb_initial = mr.FKinBody(M, Blist, thetalist)
    Vb_initial = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb_initial), Tsd)))
    wb_initial = np.linalg.norm(Vb_initial[:3])
    vb_initial = np.linalg.norm(Vb_initial[3:])

    # Format initial iteration information
    iteration_info = f"Iteration {i}:\n"
    iteration_info += f"θ^{i}: {np.round(thetalist, 3)}\n"
    iteration_info += f"Tsb(θ^{i}):\n{np.round(Tsb_initial, 3)}\n"
    iteration_info += f"Vb: {np.round(Vb_initial, 3)}\n"
    iteration_info += f"||wb||: {round(wb_initial, 3)}\n"
    iteration_info += f"||vb||: {round(vb_initial, 3)}\n"
    
    # Append initial iteration to iterationTable
    iterationTable.append(iteration_info)

    # Save initial joint vector (rounded to 3 decimal places)
    joint_vectors.append(np.round(thetalist, 3))

    Vb = Vb_initial
    err = np.linalg.norm(Vb[:3]) > eOmg or np.linalg.norm(Vb[3:]) > ev

    while err and i < maxiterations:
        thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)
        i += 1
        Tsb_current = mr.FKinBody(M, Blist, thetalist)
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb_current), Tsd)))
        wb_current = np.linalg.norm(Vb[:3])
        vb_current = np.linalg.norm(Vb[3:])
        err = np.linalg.norm(Vb[:3]) > eOmg or np.linalg.norm(Vb[3:]) > ev

        # Format current iteration information
        iteration_info = f"Iteration {i}:\n"
        iteration_info += f"θ^{i}: {np.round(thetalist, 3)}\n"
        iteration_info += f"Tsb(θ^{i}):\n{np.round(Tsb_current, 3)}\n"
        iteration_info += f"Vb: {np.round(Vb, 3)}\n"
        iteration_info += f"||wb||: {round(wb_current, 3)}\n"
        iteration_info += f"||vb||: {round(vb_current, 3)}\n"

        # Append current iteration to iterationTable
        iterationTable.append(iteration_info)

        # Save current joint vector (rounded to 3 decimal places)
        joint_vectors.append(np.round(thetalist, 3))

    success = not err
    theta_final = thetalist

    # Convert joint_vectors to a numpy array
    joint_vectors = np.array(joint_vectors)

    # Save joint_vectors to a CSV file with a fixed name
    filename = "assignment2/iterates.csv"
    np.savetxt(filename, joint_vectors, delimiter=",")

    return iterationTable, success, joint_vectors



# Given values from the example 4.5 in the book



#Initialization parameters, values in meters and radians
L1 = 0.425
L2 = 0.392
H1 = 0.089
H2 = 0.095
W1 = 0.109
W2= 0.082 

# Define the matrix M as a numpy array

M = np.array([[-1, 0, 0, L1+L2],
              [0, 0, 1, W1+W2],
              [0, 1, 0, H1-H2],
              [0, 0, 0, 1]])

# define B LIST list

Blist = np.array([[0, 1, 0, W1+W2, 0, L1+L2],
              [0, 0, 1, H2, -L1-L2, 0],
              [0, 0, 1, H2, -L2, 0],
              [0, 0, 1, H2, 0, 0],
              [0, -1, 0, -W2, 0, 0],
              [0, 0, 1, 0, 0, 0]]).T

Tsd = np.array([[0, 1, 0, -0.5],
                [0, 0, -1, 0.1],
                [-1, 0, 0, 0.1],
                [0, 0, 0, 1]])
eOmg=0.001
ev=0.0001

#my initial guess

theta0 = np.array([-4.324,  19.539, -39.188,  23.349 , -0.829 ,  0.951])



# Example usage:
[iterationTable, success, joint_vectors] = IKinBodyIterates(Blist, M, Tsd, theta0, eOmg, ev)

print("joint vector matrix from initial guess to final solution :")
print(joint_vectors)  # you could print the joint vectors


for iteration in iterationTable: # or use a for loop to print each iteraation
    print(iteration)

print("your convergence is :" +str(success))  # or see if you get convergence




