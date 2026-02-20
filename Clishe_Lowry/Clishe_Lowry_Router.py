import time
import heapq

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
    g = inf              # f(n) = g(n) + h(n) where n is the next node on the path. G is the actual cost of the path from the start node to n. initialized to infinity for all cells except start.
    h = inf              # h is a heuristic estimate of the cost of the cheapest path from n to goal. 
    g[start] = 0         # actual cost of reaching the start node from the start node
    f[start] = h[start]  
    while open_set is not empty:        # loop until a solution is found or the frontier is empty
        current  = node in open_set (frontier) having the lowest f[] value
        if current = goal:
            return reconstruct_path(came_from, current)         # reconstructs path
        open_set.remove(current)                                # current is not the goal node. we remove from the frontier 
        for each neighbor of current:
            tentative_g = g[current] + c(current,neighbor)      # cost of best known path to current + cost to step from current to neighbor. 
            if tentative_g < g[neighbor]:                       # if this new cost is smaller than the previously recorded g[neighbor], then we have found a better path to that neighbor.
                came_from[neighbor] = current                   # since we found a better path, we record that the best way to reach the neighbor is by current
                g[neighbor] = tentative_g                       # we update the cost to reach the neighbor
                f[neighbor] = tentative_g + h(neighbor)         # update the total estimated cost
                if neighbor not in open_set:                    # add the neighbor to the open set
                    open_set.add(neighbor)                                  
    
    """
    def h(current, goal):
        # h is the heurisitic estimate from current to the goal
        x1,y1 = current     # unpacks current coordinates
        x2,y2 = goal        # unpacks goal coordinates
        return abs(x1-x2) + abs(y1-y2)
    
    def reconstruct_path(move_list, current_node):
        # traces back the path from current_node back to the start of move_list
        pass

    f_start = h(start)
    g_start = 0

    open_set = []                              
    heapq.heappush(open_set, (f_start,start))   # pushing the tuple (f_start,start) to the heap open_set. the first element of each tuple determines the order in which elements are popped. 

    came_from = {start: None}                               # initializes the came_from dict that allows us to retrace steps. key: value tells us that to get to key, we came from value. 
    while open_set: 
        current = heapq.heappop(open_set)                   # removes node with lowest f from the heap and sets it to current. 
        if current == goal: 
            return reconstruct_path(came_from, current)     # if current is the goal, then we stop and reconstruct the path
        