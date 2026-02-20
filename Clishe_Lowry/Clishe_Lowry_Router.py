import time

# My initial thoughts are this: 
# I still need to re-learn the different algorithms, but I do want there to be a global routing -> detailed routing strategy.
# Therefore, I think the main routing function will take in a N x M grid (NOT the placement grid) and use one of the algorithms 
# to find a route between two selected points. By not using the original placement grid here, we will be able to use this same
# function for both global and detailed routing. There will need to be another function that, once a global route is found, transforms that global route
# into its own placement grid that we can input back into this main function and find a detailed route inside of it. 

# For routing, I will use A*. 

def A_star(start, goal): 
    #pseudocode below
    """
    open_set = {start}   # frontier implemented as a priority queue (not set) because order matters
    came_from = dict()   # came_from initialized as an empty dict
    g = inf              # f(n) = g(n) + h(n) where n is the next node on the path. G is the actual cost of the path from the start node to n
    h = inf              # h is a heuristic estimate of the cost of the cheapest path from n to goal. 
    g[start] = 0         # actual cost of reaching the start node from the start node
    f[start] = h[start]  
    while open_set is not empty:        # loop until a solution is found or the frontier is empty
        current  = node in open_set (frontier) having the lowest f[] value
        if current = goal:
            return reconstruct_path(came_from, current)         # reconstructs path
        open_set.remove(current)                                # current is not the goal node. we remove from the frontier 
        for each neighbor of current:
            tentative_g = g[current] + c(current,neighbor)      # c(n,m) is the cost of going from n to m. tentative_g analyzes the cost of reaching the neighbor from the current node
            if tentative_g < g[neighbor]:                       # if the cost of reaching that neighbor from the current node is less than the cost of reaching the neighbor from the start
                came_from[neighbor] = current                   # to get to neighbor, you came from current
                g[neighbor] = tentative_g                       
                f[neighbor] = tentative_g + h(neighbor)
                if neighbor not in open_set:
                    open_set.add(neighbor)                                  
    
    """