import time
import heapq
import math

"""
Some notes on future implementation: 
I want there to be a distinction between global routing and detailed routing. Therefore, I am thinking of implementing the workflow below:
1) Extract pin coordinates for all nets to be routed. 
2) Decide in some way on a routing order. Maybe start with longest first (by manhattan distance) or most constrained first (pins in dense regions, so more expected congestion)
3) Create some kind of routing database and maintain it as routing occurs. It should contain information for each cell that describes blockages on that cell on a particular metal layer.
   For example, a list (more likely an unordered set) with information such as (x,y,layer) that describes what layers are occuping the particular x,y coordinate. Ill need to think more on this.
4) For each net, decide whether to run global routing first. If not, skip directly to detailed routing.
5) Global routing: 
    * Coarsen placement grid into tiles containing k*k cells
    * Run 2D A* routing on the tile grid from the source tile to the target tile.
    * If a tile contains many previously routed nets, maybe consider this tile as crowded and store some kind of congestion penalty to this tile in the database on step 3. 
    * Maybe add turn penalties. Global routing is going to occur in 2D, but we know that if there are many turns, we need at least that many vias, so a turn penalty here is a kind of proxy for vias. 
6) Convert global route into a corrdior on the detailed grid
    * Expand the tile path from global routing into a set of preferred cells
    * Add cost penalties for cells outside the corrdior
7) Detailed Routing: 
    * Route on a 3D grid (something like (x,y,layer)) across m2-m9 with pins on m1. 
    * Legal moves will only be steps within the same metal layer in the preferred direction plus via moves between adjacent layers
    * Final path must start and end with m1-m2 vias. Each cell hop will be 1 + addl' cost (from global routing corrdior. maybe additional costs will be added to congested cells).
      Each via will add 2. 
8) Put routed path into routing database from step 3. Add occupancy details, update congestion, that kind of thing. Clear global routing corrdiors. 
9) If severe congestion or routing failures, rip up and reroute selected nets (the ones causing the blockage for example. Might need to rewatch this portion of the lecture (week 6) for additional info).

"""


def dist(c1, c2):
        #returns the manhattan distance between coordinates c1 and c2
        x1,y1 = c1     # unpacks current coordinates
        x2,y2 = c2     # unpacks goal coordinates
        return abs(x1-x2) + abs(y1-y2)

def A_star(start, goal): 
    # Below are some helper functions for A*. In the future, it might be beneficial to define these functions outside of A_star(). If A_star() is called
    # frequently, then defining these functions globally instead of locally will likely save some time (the functions will only be defined once rather than on each iteration).
    # For now though, I will keep them local for the sake of readability. 
      
    def h(current, goal):
        # h is the heurisitic estimate from current to the goal
        # A better heuristic other than dist may be looked into, but keep in mind that h() should be consistent. Non consistent heuristics might require a rewrite of the A* algorithm
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
        # Before this function can be implemented, we need to know what the structure of current_cell is. A coordinate? A cell name? Something else? 
        # We will also need to figure out the structure of the placement grid. find_neighbors should not return neighbors that are outside the grid or that are blocked. 
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