import numpy as np
from scipy.optimize import linprog
import matplotlib.pyplot as plt

# Clear the output log file at the start
with open('output_log.txt', 'w') as f:
    f.write("")  # This ensures the file is cleared at the beginning

class StaticMass:
    def __init__(self, mass, x, y):
        self.mass = mass  # Mass of the body
        self.x = x        # X position of the center of mass
        self.y = y        # Y position of the center of mass

class Contact:
    def __init__(self, body_a, body_b, x_contact, y_contact, normal_angle, friction_coefficient):
        self.body_a = body_a               # The first body in the contact
        self.body_b = body_b               # The second body in the contact
        self.x_contact = x_contact         # X position of the contact point
        self.y_contact = y_contact         # Y position of the contact point
        self.normal_angle = normal_angle   # Normal angle of contact
        self.friction_coefficient = friction_coefficient  # Friction coefficient

def checkForStability(m, contacts):
    g = 9.81
    num_bodies = len(m)
    for i in range(num_bodies):
        contact_bodies = []  # Initialize the contact bodies list
        
        for contact in contacts:     # Loop over all contacts to find the ones related to current body (i + 1)
            if contact.body_a == (i + 1) or contact.body_b == (i + 1):
                contact_bodies.append(contact)
        
        num_k = len(contact_bodies) * 2   # Friction cone has two edges per contact
        f = np.ones(num_k)  # Objective function (minimizing sum of contact forces)
        A_ub = -np.identity(num_k)  # Inequality constraints: forces must be non-negative
        b_ub = -np.ones(num_k)  # Vector for inequality constraints (forces >= 0)
        F = [] # Initialize matrix to store friction cone directions

        for contact in contact_bodies:
            x = contact.x_contact  # Contact point x-coordinate
            y = contact.y_contact  # Contact point y-coordinate 
            ang = contact.normal_angle  # Contact normal angle
            
            # Adjust angle if the current body is body_b
            if contact.body_b == (i + 1):
                ang -= np.pi  # Reverse the angle if body_b is the current body
            
            friction_coeff = contact.friction_coefficient  # Friction coefficient

            # Compute the angle of the friction cone edges
            theta = np.arctan2(friction_coeff, 1)  # Friction cone angle

            # Calculate the two force directions (the edges of the friction cone)
            f1 = [np.sin(ang + theta) * x - np.cos(ang + theta) * y,
                  np.cos(ang + theta),
                  np.sin(ang + theta)]
            f2 = [np.sin(ang - theta) * x - np.cos(ang - theta) * y,
                  np.cos(ang - theta),
                  np.sin(ang - theta)]
            
            F.extend([f1, f2])

        # Convert F to a numpy array and transpose it
        A_eq = np.array([np.array(xi) for xi in F]).T
        static_mass = m[i]
        b_eq = [static_mass.mass * static_mass.x * g, 0, static_mass.mass * g]  # Corrected

        # Solve the linear program
        result = linprog(c=f, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, method='highs')
        
        if not result.success:  # If any body fails, the system collapses
            return result  # Return the result for testing
        

    return result  # If all bodies are stable, return the last result


def log_results(example_number, masses, contacts, result):
    """Logs the results to an output file and prints them to the terminal."""

    result_log = f"Example {example_number}:\n"
    
    # Log the masses
    result_log += "Masses:\n"
    for i, mass in enumerate(masses, start=1):
        result_log += f"Body {i}: Mass = {mass.mass}, Center of Mass = ({mass.x}, {mass.y})\n"

    # Log the contacts
    result_log += "\nContacts:\n"
    for j, contact in enumerate(contacts, start=1):
        result_log += (f"Contact {j}: Body A = {contact.body_a}, Body B = {contact.body_b}, "
                       f"Contact Point = ({contact.x_contact}, {contact.y_contact}), "
                       f"Normal Angle = {contact.normal_angle:.2f}, "
                       f"Friction Coefficient = {contact.friction_coefficient:.2f}\n")

    # Log the result
    if result.success:
        result_log += "\nThe assembly remains standing.\n"
    else:
        result_log += "\nThe assembly would collapse.\n"
    
    result_log += "-" * 40 + "\n"

    # Write to file
    with open('output_log.txt', 'a') as f:
        f.write(result_log)
    
    # Print to terminal
    print(result_log)

def main():

    # Example 1: Collapsing assembly
    result1 = checkForStability(m1, contacts1)
    log_results(1, m1, contacts1, result1)

    # Example 2: Stable assembly
    result2 = checkForStability(m2, contacts2)
    log_results(2, m2, contacts2, result2)




# Example where system collapses

m1 = [
    StaticMass(2, 1, 1.5),  # Body 1 (x, y of center of mass)
    StaticMass(4, 7, 1)     # Body 2 (x, y of center of mass)
]
contacts1 = [
    Contact(1, 0, 1, 0, np.pi/2, 0.4),  # Body 1 with ground (0)
    Contact(1, 2, 4, 4, np.pi/4, 0.1),  # Body 1 with Body 2 
    Contact(2, 0, 5, 0, np.pi/2, 0.4),  # Body 2 with ground (0)
    Contact(2, 0, 9, 0, np.pi/2, 0.4)   # Body 2 with ground (0)
]



# Example where system remains stable (the only thing i changed was the friction coefficient between body 1 and 2)

m2 = [
    StaticMass(2, 1, 1.5),  # Body 1 (x, y of center of mass)
    StaticMass(4, 7, 1)     # Body 2 (x, y of center of mass)
]
contacts2 = [
    Contact(1, 0, 1, 0, np.pi/2, 0.4),  # Body 1 with ground (0)
    Contact(1, 2, 4, 4, np.pi/4, 0.3),  # Body 1 with Body 2 
    Contact(2, 0, 5, 0, np.pi/2, 0.4),  # Body 2 with ground (0)
    Contact(2, 0, 9, 0, np.pi/2, 0.4)   # Body 2 with ground (0)
]



main()  # run the code


