import numpy as np
import csv
import heapq
import os

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



# Example code
num_nodes = 12
start_node = 1
goal_nodes = {12}

A_star_search(num_nodes, start_node, goal_nodes)
