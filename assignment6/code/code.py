import numpy as np
from scipy.optimize import linprog

# Clear the output log file at the start
with open('output_logs.txt', 'w') as f:
    f.write("")  # This ensures the file is cleared at the beginning

def is_form_closed(contact_points, normal_angles):
    N = contact_points.shape[0]
    
    # Compute contact normals from angles
    contact_normals = np.zeros_like(contact_points)
    for i in range(N):
        contact_normals[i, 0] = np.cos(normal_angles[i])
        contact_normals[i, 1] = np.sin(normal_angles[i])
    
    # Compute the wrench matrix F
    F = np.zeros((3, N))
    for i in range(N):
        torque = contact_points[i, 0] * contact_normals[i, 1] - contact_points[i, 1] * contact_normals[i, 0]
        F[0, i] = torque
        F[1:, i] = contact_normals[i, :]
    
    # Define the parameters for linprog
    f = np.ones(N)
    A_ub = -np.eye(N)
    b_ub = -np.ones(N)
    A_eq = F
    b_eq = np.zeros(3)
    
    # Solve the linear program
    result = linprog(c=f, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, method='highs')
    
    # Return results
    if result.success:
        return True, result.x
    else:
        return False, []

def log_results(example_number, contact_points, normal_angles, form_closed, k):
    """Logs the results to an output file and prints them to the terminal."""
    result_log = f"Example {example_number}:\n"
    result_log += f"Contact Points: {contact_points.tolist()}\n"
    result_log += f"Normal Angles: {normal_angles.tolist()}\n"
    if form_closed:
        result_log += f"Form closure is given with k = {k}\n"
    else:
        result_log += "Form closure is not given\n"
    result_log += "-" * 40 + "\n"
    
    # Write to file
    with open('output_logs.txt', 'a') as f:
        f.write(result_log)
    
    # Print to terminal
    print(result_log)

# Example 1: In Form Closure
contact_points_1 = np.array([[1, 0], [3, 0], [2.5, 2], [1.5, 2]])
normal_angles_1 = np.array([np.pi/2, np.pi/2, -np.pi/2, -np.pi/2])
form_closed_1, k_1 = is_form_closed(contact_points_1, normal_angles_1)

log_results(1, contact_points_1, normal_angles_1, form_closed_1, k_1)

# Example 2: Not in Form Closure
contact_points_2 = np.array([[1, 0], [3, 0], [2.5, 2], [1.5, 2]])
normal_angles_2 = np.array([np.pi/2, np.pi/2, -np.pi/4, -np.pi/4])  # Altered normals
form_closed_2, k_2 = is_form_closed(contact_points_2, normal_angles_2)

log_results(2, contact_points_2, normal_angles_2, form_closed_2, k_2)
