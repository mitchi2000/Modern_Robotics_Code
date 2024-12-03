import numpy as np
import os
import csv
import random
import math
import heapq

def get_obstacles(script_dir):
    file_path = os.path.join(script_dir, 'results', 'obstacles.csv')
    obstacles_list = []
    with open(file_path, mode='r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x = float(row[0].strip())
            y = float(row[1].strip())
            diameter = float(row[2].strip())
            radius = diameter / 2.0  # Convert diameter to radius
            radius += robot_width / 2.0  # Add half of the robot's width to the radius
            obstacles_list.append((x, y, radius))
    
    return obstacles_list

def is_inside_obstacle(x, y, obstacles):
    for obs_x, obs_y, radius in obstacles: # check if chosen ositions are within obstacles
        distance = np.sqrt((x - obs_x) ** 2 + (y - obs_y) ** 2)
        if distance <= radius:
            return True
    return False

def get_Nodes(num_nodes, xMin, xMax, yMin, yMax, obstacles_list):
    random_nodes = []

    # Add the start point
    heuristic_start = round(np.sqrt((xMin - xMax) ** 2 + (yMin - yMax) ** 2), 4)
    random_nodes.append((1, xMin, yMin, heuristic_start))
    counter = 2

    while len(random_nodes) < num_nodes:
        x = round(random.uniform(xMin, xMax), 1)
        y = round(random.uniform(yMin, yMax), 1)

        # Check if this (x, y) is already in the list of nodes
        if any(x == node[1] and y == node[2] for node in random_nodes):
            continue
        if not is_inside_obstacle(x, y, obstacles_list): # check if node is inside of obstacle
            heuristic_ctg = np.sqrt((x - xMax) ** 2 + (y - yMax) ** 2)
            heuristic_ctg = round(heuristic_ctg, 4)
            random_nodes.append((counter, x, y, heuristic_ctg))
            counter += 1
    
    # Add the goal point
    random_nodes.append((num_nodes + 1, xMax, yMax, 0))

    # Write the coordinates to the CSV file
    nodes_file_path = os.path.join(script_dir, 'results', 'nodes.csv')
    with open(nodes_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(random_nodes)
    
    return random_nodes

def get_edges(nodes, k):
    edges = []
    for i, node in enumerate(nodes): # calculate the distance between each node and every other node
        distances = []
        for j, other_node in enumerate(nodes):
            if i != j:
                dist = np.sqrt((node[1] - other_node[1]) ** 2 + (node[2] - other_node[2]) ** 2)
                distances.append((dist, other_node[0]))  # (distance, node_id)

        distances.sort(key=lambda x: x[0]) # sort the list and choose k closest neighbors
        for dist, neighbor_id in distances[:k]:
            edges.append((node[0], neighbor_id, round(dist, 4)))

    return edges

def is_in_c_free(x, y, obstacles_list):

    for (obs_x, obs_y, obs_radius) in obstacles_list: #Checks if a given point is in C_free (not within any obstacle)
        dist = math.dist((x, y), (obs_x, obs_y))
        if dist <= obs_radius:
            return False
    return True  # return which are inside and which are outside

def is_collision_free(node_1, node_2, obstacles_list):
    #Checks if the path between two nodes is not in collision with any obstacle
    x1, y1 = node_1[1], node_1[2]
    x2, y2 = node_2[1], node_2[2]

    # Vector from node1 to node2
    dx = x2 - x1
    dy = y2 - y1

    # 20 random values between 0 and 1
    scalars = np.random.uniform(size=20)

    # Sample 20 points on the path between node1 and node2
    x = [x1 + t*dx for t in scalars]
    y = [y1 + t*dy for t in scalars]

    # Check if each sampled point is in C_free
    for x_i, y_i in zip(x, y):
        if not is_in_c_free(x_i, y_i, obstacles_list):
            return False
    
    return True

def get_valid_edges(edges, nodes, obstacles_list):
    valid_edges = []
    for (node1, node2, distance) in edges:
        node1_data = nodes[node1 - 1] # from edges 1,12 i get the data for both of these nodes
        node2_data = nodes[node2 - 1]
        
        if is_collision_free(node1_data, node2_data, obstacles_list): #send these data to check if they are collision free
            valid_edges.append((node1, node2, distance))
    
    edges_file_path = os.path.join(script_dir, 'results', 'edges.csv')
    with open(edges_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(valid_edges)

    return valid_edges


def A_star_search(num_nodes, start_node, goal_nodes):

    # Get the directory of the script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Define file paths 
    edges_file_path = os.path.join(script_dir, 'results', 'edges.csv')
    nodes_file_path = os.path.join(script_dir, 'results', 'nodes.csv')
    path_file_path = os.path.join(script_dir, 'results', 'path.csv')
    

    with open(nodes_file_path, mode='r') as file:  # Open the CSV file of nodes.csv
        csv_reader = csv.reader(file)
        heuristic_costs = []
        for row in csv_reader:
            if row:
                heuristic_cost = float(row[3])
                heuristic_costs.append(heuristic_cost)

    with open(edges_file_path, mode='r') as file:  # Open the CSV file of edges.csv
        csv_reader = csv.reader(file)
        edges = []
        for row in csv_reader:
            edges.append(row)

    # Initialize the cost matrix with -1 (no edge)
    cost_matrix = [[-1] * (num_nodes + 1) for _ in range(num_nodes + 1)]

    # Fill the matrix with the costs from the edges
    for edge in edges:
        node1 = int(edge[0])
        node2 = int(edge[1])
        cost = float(edge[2])
        cost_matrix[node1][node2] = cost
        cost_matrix[node2][node1] = cost  # Make it symmetric

    for i in range(1, num_nodes + 1):  # Set diagonal to -1 (no cost to move from a node to itself)
        cost_matrix[i][i] = -1

    # Initialize past_cost array
    past_cost = [float('inf')] * (num_nodes + 1)  # set initial past cost of everything else to infinity
    past_cost[start_node] = 0  # Cost to reach the start node (node 1) is 0

    # Initialize the parent array
    parent = ['-'] * (num_nodes + 1)

    # Initialize the OPEN list with the start node
    OPEN = []
    heapq.heappush(OPEN, (past_cost[start_node] + heuristic_costs[start_node - 1], start_node))  # (estimated_total_cost, node)

    # Initialize the CLOSED set
    CLOSED = set()

    # A* Search Algorithm
    while OPEN:
        
        _, current = heapq.heappop(OPEN)  # Step 1: Remove the node with the lowest estimated total cost from OPEN

        CLOSED.add(current)  # Step 2: Add the node to CLOSED
        
        if current in goal_nodes:  # Step 3: Check if `current` is in the goal set
            print("Goal node reached:", current)
            
            # Reconstruct the path from the start node to the goal node
            path = []
            while current != '-':  # Start node is initialized with '-'
                path.append(current)
                current = parent[current]
            path.reverse()  # Reverse the path to get the correct order
                       
            print("Path found:", path)

            # Save the path to path.csv in the results folder
            with open(path_file_path, mode='w', newline='') as file:
                csv_writer = csv.writer(file)
                csv_writer.writerow(path)  # Write all nodes on the same line
            
            print("Path saved to path.csv")
            return path  # Return path after saving
        
        # Update neighbors
        for nbr in range(1, num_nodes + 1):
            if cost_matrix[current][nbr] != -1 and nbr not in CLOSED:  # check to see if there is an edge between 2 nodes
                
                tentative_past_cost = past_cost[current] + cost_matrix[current][nbr]  # Calculate tentative cost to reach neighbor
                
                if tentative_past_cost < past_cost[nbr]:  # If the new path is better, update past_cost and parent
                    past_cost[nbr] = tentative_past_cost
                    parent[nbr] = current
                    estimated_total_cost = past_cost[nbr] + heuristic_costs[nbr - 1]  # Calculate the estimated total cost for the neighbor
                    heapq.heappush(OPEN, (estimated_total_cost, nbr))  # Add or update the neighbor in OPEN
        
    # If OPEN is empty and no goal node is found, the path does not exist
    print("No solution found.")
    return None  # Return None if no path is found





script_dir = os.path.dirname(os.path.abspath(__file__)) # indicate the folder's directory
robot_width = 0.05 # remove the robot's width from other calculations
xMin, xMax = -0.5, 0.5 # define the boundaries of the space
yMin, yMax = -0.5, 0.5 
num_nodes = 30 # define how many sample nodes you need
start_node = 1 # define start node
goal_nodes = {num_nodes+1} # define goal node
k = 7 # number of neighbors I want to have for each node
random.seed(18) # choose a consistent pattern of nodes # 33 was my hm

# main
obstacles = get_obstacles(script_dir) 
nodes = get_Nodes(num_nodes, xMin, xMax, yMin, yMax, obstacles)
edges = get_edges(nodes, k)
validEdges = get_valid_edges(edges, nodes, obstacles)
A_star_search(num_nodes+1, start_node, goal_nodes)

