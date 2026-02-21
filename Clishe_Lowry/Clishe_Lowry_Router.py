import time
import heapq
import math

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
    def dist(c1, c2):
        #returns the manhattan distance between coordinates c1 and c2
        x1,y1 = c1     # unpacks current coordinates
        x2,y2 = c2     # unpacks goal coordinates
        return abs(x1-x2) + abs(y1-y2)
    
    def h(current, goal):
        # h is the heurisitic estimate from current to the goal
        # A better heuristic other than dist may be looked into, but keep in mind that h() should be consistent, or at least admissible. 
        return dist(current,goal)
    
    def reconstruct_path(came_from, current):
        # traces back the path from current_node back to the start of move_list. Successive indices in the output list indicate successive nodes in the path. 
        path = [current]                            # start at the goal node (this function is only called when current == goal)
        while came_from[current] is not None:       # came_from is the list of key: value pairs where in order to get to key, we traversed through value. came_from[start] == None
            current = came_from[current]            # moving down the list toward the start
            path.append(current)                    
        path.reverse()                              # reverse the list to get a start --> goal path instead of a goal --> start path. 
        return path


    def find_neighbors(current_cell):
        # returns a list of the neighbors of current_cell 
        pass

    g = {start: 0.0}                            # initializing the g dictionary. Each cell other than the start is initialized to infinity 

    open_set = []                              
    heapq.heappush(open_set, (h(start,goal),start))  # pushing the tuple (f_start,start) to the heap open_set. the first element of each tuple determines the order in which elements are popped. 

    closed = set()                              # unordered set of nodes already expanded (popped and processed). With a consistent heuristic, once a node is expanded its best g is final, so we never expand it again. This is needed if we allow duplicates in open_set (we do)

    came_from = {start: None}                               # initializes the came_from dict that allows us to retrace steps. key: value tells us that to get to key, we came from value. 
    while open_set: 
        current = heapq.heappop(open_set)[1]                # removes node with lowest f from the heap and sets it to current. heappop returns the (f,node) tuple at the top of the heap, but we only need the node, so we only keep tupl0e[1]
        
        # if the node is in current, then we have already visited that node. As long as the heuristic is consistent, we know that since we already visited the node, the g value it had is guaranteed to be optimal, so we have no need to revisit it (executing the rest of the loop)
        if current in closed:
            continue
        
        if current == goal: 
            return reconstruct_path(came_from, current)     # if current is the goal, then we stop and reconstruct the path
        
        closed.add(current)                                 # by popping current from open_set, we are visiting the node. So we add it to closed.
        
        for neighbor in find_neighbors(current):
            tentative_g = g[current] + dist(current,neighbor)
            
            if tentative_g < g.get(neighbor, math.inf):          # this is where the "each cell other than the start is initialized to infinity" comes into play. if current does not yet exist in the g dict, then we use infinity as a placeholder. 
                came_from[neighbor] = current
                g[neighbor] = tentative_g
                f_neighbor = tentative_g + h(neighbor, goal)
                heapq.heappush(open_set, (f_neighbor, neighbor))   # we always push neighbor to the frontier. This allows duplicates, but old values of neighbor that are already in open_set with larger f[neighbor] values will never be visited due to the "if node in closed" check at the start of the loop

    print(f"A* terminated with no path found between {start} and {goal}.")  # in the future I may raise an error here instead of a print statement and return None.
    return None