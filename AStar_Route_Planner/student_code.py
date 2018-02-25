from helpers import show_map
from math import sqrt



def shortest_path(M,start,goal):
    '''
    Shortest Path:
    Implements A* Algorithm
    
    
    *************************************
    Udacity Lecture: 
        
    Minimise value: f = g + h
    g(path) = path cost
    h(path) = h(state) = estimated distance to goal (straight line distance)
    f(path) = total path cost
    
    minimising g, helps keep the path short
    minimising h, helps keep focused on goal
    result: search strategy that is the best possible. Finds the shortest length path while expanding minimum number of paths possible
    
    Definition of a Problem:
    Initial state -> s0
    Actions (s) -> {a1, a2, a3...}
    Result (s,a) -> s'
    GoalTest (s) -> Bool (True|False)
    PathCost ( si > aj > si+1 > aj+1 ....) -> cost value (n)
    StepCost (s, a, s') -> n (cost of action)
        
    ****************************************
    Reference:
    
    A* Search Algorithm (Wikipedia):
    https://en.wikipedia.org/wiki/A*_search_algorithm
    
    Introduction to A*:
    https://www.redblobgames.com/pathfinding/a-star/introduction.html
    
    Pathfinding using A*:
    http://web.mit.edu/eranki/www/tutorials/search/
    
    Finding max, min values of a multi-dimensional dictionary:
    https://stackoverflow.com/questions/20990282/find-max-min-of-values-in-multidimensional-dict
    
    Return key associated with min value of a dict if min key is not in set: 
    https://stackoverflow.com/questions/40668338/return-key-associated-with-min-value-of-a-dict-if-min-key-is-not-in-set
    
    *****************************************
    
    
    Parameters:
        M: Input Graph (Map)
        start: start node 
        goal: goal node
    Returns:
        path: shortest path from start to goal (list)
    
    '''
    
    print("Shortest path called")
    
    # Printing and debugging flags
    printing = True
    print("Print Mode: ", printing)
    
    debugging = False
    print("Debug Mode: ", debugging)
    
    
    # Initialise fronter, explored sets
    frontier = set([start])
    explored = set([])
    
    
    # Create dictionary to hold details of nodes on graph:    
        # Previous: best previous node to get to the node (key)
        # gCost: path cost of reaching the node (key). Default value 1000
        # hCost: heuristic (straight line distance to goal) of the node (key). Default value 1000
        # totalCost: totalCost (f = g + h) of the node (key). Default value 10000
    
    nodes = {}
    for key in M.intersections.keys():
        nodes[key] = {}
        nodes[key]["previous"] = None
        nodes[key]["gCost"] = 1000
        nodes[key]["hCost"] = 1000
        nodes[key]["totalCost"] = 10000
    

    # Determine x, y co-ordinates of start and goal nodes
    xS, yS = M.intersections[start][0], M.intersections[start][1]
    xG, yG = M.intersections[goal][0], M.intersections[goal][1]
    

    # Set details of start node
    nodes[start]["previous"] = start
    nodes[start]["gCost"] = 0
    nodes[start]["hCost"] = distanceBetween(xS, yS, xG, yG)
    nodes[start]["totalCost"] = distanceBetween(xS, yS, xG, yG)
    

    # Debugging: List to track nodes that have been visited, used for debugging
    if debugging:
        currentList = [start]
        
    # Continue while the frontier still contains nodes
    while bool(frontier):
        
        # Select the node with minimum total cost that has not been explored
        # (Return key of minimum value of totalCost (value), where keys are contained (intersection) in frontier)
                
        current = min(set(nodes).intersection(frontier), 
                       key=lambda k:float(nodes[k]["totalCost"]))
        
        
        # Debugging: If current node has not been previously evaluated, add to CurrentList
        if debugging:
            if current not in currentList:
                currentList.append(current)
        
        # If the current node is the goal, search finished, return full path
        if current == goal:
            
            print("Found Goal node")
            path = reconstructPath(nodes, current, start)
            print(path)
            
            if printing: 
                print("Path Distance: ", pathCost(M, path))
                print("Straight Line Distance: ", distanceToGoal(M, start, goal))
                show_map(M, start=start, goal=goal, path=path)
            
            
            # Debugging: returns all nodes evaluated
            if debugging:
                print("Nodes travelled: ", currentList)
                printNodes(nodes)
            
            return path
        
        # Remove current node from the frontier set, add to explored set
        frontier.remove(current)
        explored.add(current)
        
        # Evaluate each node (road) attached to current node         
        for i in M.roads[current]:
            
            # Only evaluate unexplored paths
            if i not in explored:
                                
                # Add node to frontier if not on
                if i not in frontier:
                    frontier.add(i)
                
                # Calculate potential total cost of the step on path
                tempCost = nodes[current]["gCost"] + stepCost(M, current, i) + distanceToGoal(M, i, goal)
                
                # If potential total cost is less than current total cost for node (i.e. better path), update node details
                if tempCost < nodes[i]["totalCost"]:
                                        
                    nodes[i]["previous"] = current
                    nodes[i]["gCost"] = nodes[current]["gCost"] + stepCost(M, current, i)
                    nodes[i]["hCost"] = distanceToGoal(M, i, goal)
                    nodes[i]["totalCost"] = nodes[i]["gCost"] + nodes[i]["hCost"]
                    
                    # Debugging: See each node being evaluated
                    if debugging:
                        print("Better path found")
                        print("Node: ", i, nodes[i])
      
    return


def printNodes(nodes):
    '''
    Helper function to print all nodes, for debugging
    '''
    
    for node in nodes:
        print(nodes[node])

def reconstructPath(nodes, current, start):
    '''
    Helper function to recreate full path by assessing current, previous and starting nodes.
    
    
    Parameters:
        nodes: dictionary containing node details
        current: node currently being investigated in graph
        start: starting node
        
    Returns:
        totalPath: complete path from start node to current node
        
    '''
    
    # Add the current node to the path
    totalPath = [current]
    
    # Continue while current node is in previous
    while current in nodes:
        
        # Change the current node to previous node
        current = nodes[current]["previous"]
        
        # Add the current node to the path, if it isnt already added
        if current not in totalPath:
            totalPath.insert(0, current)
        
        # Stop when the current node being assessed is the start node
        if current == start:
            break
    
    return totalPath
    

def stepCost(M, nodei, nodej):
    """Returns the cost of a step from node i to node j. 
    Assumes distance between nodes is a straight line.
    
    Parameters:
        M = input graph (map)
        nodei: number of node i in graph
        nodej: number of node j in graph
    Returns:
        cost: straight line distance (cost) between two nodes
    
    """
    # Extract x, y co-ordinates of nodes
    xi, yi = M.intersections[nodei][0], M.intersections[nodei][1]
    xj, yj = M.intersections[nodej][0], M.intersections[nodej][1]
    
    # Return straight line distance between
    return distanceBetween(xi, yi, xj, yj)
    
    

def pathCost(M, path):
    """ Returns the total cost of a path
    
    Parameters:
        M: input graph (map)
        path: node path (list)
    Returns:
        totalPathCost: sum of individual step costs of path
    """
    totalPathCost = 0
    
    for i in range(len(path) - 1):
        # print("Path: ", path)
        # print("Step on path: ", path[i])
                
        nodei = path[i]
        nodej = path[i+1]
        totalPathCost += stepCost(M, nodei, nodej)
    
    return totalPathCost
        
        
def distanceToGoal(M, nodei, goal):
    """Returns the straight line distance from the node to goal node
    
    Parameters:
        M: input graph (map)
        nodei: number of node i in graph
        goal: goal node in graph
    Returns:
        goalDistance: straight line distance from current node to goal node
    
    """
    xi, yi = M.intersections[nodei][0], M.intersections[nodei][1]
    xG, yG = M.intersections[goal][0], M.intersections[goal][1]
    
    return distanceBetween(xi, yi, xG, yG)



def distanceBetween(xi, yi, xj, yj):
    """Returns the straight line distance to the between nodes:
    Parameters:
        xi: x co-ordinate of current node
        yi: y co-ordinate of current node
        xj: x co-ordinate of goal node
        yj: y co-ordinate of goal node
    Return:
        straight line distance from current node to another node
    """
    
    return sqrt((xi - xj)**2 + (yi - yj)**2)


   
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    