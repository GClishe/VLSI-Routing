import time
import heapq
import math
from bitarray import bitarray

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

class RoutingDB:
    """
    Creates a routing database that will be maintained as we go. To start with, it will be responsible for tracking
    grid occupancy per (x,y,layer), storing committed net routes (in case we need to rip them up), and checking for 
    legality, cost, and congestion. 
    
    """
    def __init__(self, grid_size: int, num_layers: int, tile_size: int = 10):
        self.grid_size  = grid_size             # size of the detailed grid
        self.num_layers = num_layers            # number of routing layers
        self.tile_size  = tile_size             # size of the global routing tiles

        # now we create the detailed occupancy grid. we could store occupancy details in a 3D array (each cell indexed with occ[layer][x][y]), but
        # this would be extremely inefficient. Instead, we can flatten everything to a 1D array and find a mapping between (layer,x,y) to a single integer
        # index. The mapping idx = (layer * W + x) * W + y does exactly that. Layers are stored sequentially and in row-major order.
        N = num_layers * self.grid_size * self.grid_size          # N computes the total number of bits we need to store.
        self.occ = bitarray(N)                          # creates a bitarray of size N
        self.occ.setall(0)                              # initializes all bits to 0

        # we now need to figure out how many tiles will be present in global routing.
        # with a tile size of 10, a grid with grid_size of 100 will have 10 tiles per row.
        # if the tiles of our specified size do not cleanly fit in the grid, we use ceiling function to ensure
        # that edge cells are still included in a tile, even if those tiles are smaller than the rest (they will be clipped by in_bounds checks).
        self.num_tiles = math.ceil(grid_size/tile_size)

        # now we implement global routing tile congestion ideas. the congestion of a tile is how many nets have a route that passes through the tile.
        # congestion value is initialized to 0 for all tiles
        self.tile_cong = [[0 for _ in range(self.num_tiles)] for _ in range(self.num_tiles)]

        self.net_routes = {}        # dict that contains net_name -> list of bitarray indices for cells on that net
              
    def coordinate_to_idx(self, x: int, y: int, layer: int) -> int:
        # helper function to convert (x,y,layer) to occ index.
        return (layer * self.grid_size + x) * self.grid_size + y 
    
    def idx_to_coordinate(self, idx: int) -> tuple[int, int, int]:
        layer = idx // (self.grid_size * self.grid_size)
        rem = idx % (self.grid_size * self.grid_size)
        x = rem // self.grid_size
        y = rem % self.grid_size
        return (x, y, layer)

    def in_bounds(self, x: int, y: int, layer: int) -> bool:
        # returns true if the provided coordinate is in bounds (within grid and on correct metal layer)
        return (0 <= x <= self.grid_size-1) and (0 <= y <= self.grid_size-1) and (0 <= layer < self.num_layers)

    def get_tile(self, x: int, y: int, layer: int) -> tuple[int,int]:
        # returns the tile index for the provided coordinate. This is the global routing tile that
        # the selected coordinate lies in. I expect that this will be helpful for global routing and 
        # congestion checks. 
        if not (self.in_bounds(x,y,layer)):
            raise ValueError("Coordinate given is not in-bounds.")
        
        return (x // self.tile_size, y // self.tile_size)

    def is_free(self, x: int, y: int, layer: int) -> bool:
        # checks whether (x,y,layer) is usable (not blocked)
        if not self.in_bounds(x,y,layer):
            return False
        idx = self.coordinate_to_idx(x,y,layer)                                                     # converts (x,y,layer) coordinate to a 1D index 
        return (not self.occ[idx])                                                                  # self.occ(idx) checks if the coordinate is occupied. If it is, the cell is not free. If it is not, the cell is free. Also performs bounds checks.

    def via_allowed(self, x: int, y: int, layer: int) -> bool:
        # check whether via is allowed on (x,y) to go from layer `layer` to `layer+1`
        pass

    def commit_route(self, net_name:str, path: list[tuple[int, int, int]]):
        # commit a detailed route to the database. Paths are lists of (x,y,layer) pairs.
        # note that the final export will have a different path structure
        path_indices = []                                            # list containting the bitarray indices occupied by this net
        for (x, y, layer) in path:
            if not self.in_bounds(x, y, layer):
                raise ValueError(f"Coordinate out of bounds: {(x, y, layer)}")

            idx = self.coordinate_to_idx(x, y, layer)
            path_indices.append(idx)
            if self.occ[idx]: 
                raise ValueError("collision")
            self.occ[idx] = 1

        self.net_routes[net_name] = path_indices                                             
        return None                                                  # not necessary, but return None helps with readability

    def tiles_on_net(self, net_name: str) -> list[tuple[int,int]]:
        # returns the global-routing tiles (tx, ty) that the committed route for `net_name` passes through
        cell_indices = self.net_routes.get(net_name)
        if not cell_indices:
            return []

        tile_indices: list[tuple[int, int]] = []
        seen: set[tuple[int, int]] = set()

        for cell_idx in cell_indices:
            x, y, layer = self.idx_to_coordinate(cell_idx)  # idx -> (x,y,layer)
            tx, ty = self.get_tile(x, y, layer)             # (x,y,layer) -> (tx,ty)

            tile = (tx, ty)
            if tile not in seen:
                seen.add(tile)
                tile_indices.append(tile)

        return tile_indices

    def rip_up(self, net_name: str):
        # removes a previously committed route
        indices = self.net_routes[net_name]             # gets the list of indices for cells on the net
        for idx in indices:                 
            self.occ[idx] = 0                           # sets that list element to 0
        
        affected_tiles = self.tiles_on_net(net_name)    # gets the tiles that this net passed through

        for (tx, ty) in affected_tiles:                     # subtracts 1 from the congestion value of each net that was ripped up
            if self.tile_cong[tx][ty] == 0:
                raise ValueError("Tile congestion value cannot be negative")
            else:
                self.tile_cong[tx][ty] -= 1
        
        self.net_routes.pop(net_name)                   # removes the net_name:indices item from the net_routes dict

    def congestion_penalty(self, x: int, y: int) -> float:
        # returns the congestion penalty for a detailed cell. I do not know how this will be implemented. May be removes later
        pass

    def step_cost(self, x: int, y: int, layer: int, nx: int, ny: int, nlayer: int) -> float:
        # computes incremental cost for moving from (x,y,layer) to (nx,ny,nlayer)
        # this will be helpful if we decide to introduce costs for making specific moves.
        # for example, if a step always has a cost of 1, we can just return 1. But if nlayer changes,
        # we can add 2 to the cost. Somehow this will also need to incorporate cost from leaving a cooridor 
        # created by global routing. 

        cost = 1.0                                  # base cost. each step has a cost of at least 1

        if layer != nlayer:
            cost += 2.0 * abs(nlayer - layer)       # vias cost 2 for each layer traversal. Since we can stack vias, we need to compute how many layers are traversed. 

        cost += self.congestion_penalty(nx, ny)     # add a congestion penalty

        return cost


#pasting an example netlist for testing purposes. Will delete later. 
netlist = {   'grid_size': 100,
    'nets': {   'NET_0': {   'length': 3,
                             'pins': [(99, 56), (99, 59)],
                             'type': 'LOCAL'},
                'NET_1': {   'length': 48,
                             'pins': [(5, 72), (44, 63)],
                             'type': 'MEDIUM'},
                'NET_2': {   'length': 52,
                             'pins': [(43, 74), (5, 60)],
                             'type': 'LONG'}}}


def dist(c1, c2):
        #returns the manhattan distance between coordinates c1 and c2
        x1,y1 = c1     # unpacks current coordinates
        x2,y2 = c2     # unpacks goal coordinates
        return abs(x1-x2) + abs(y1-y2)

def create_routing_order(netlist: dict) -> list[str]:
    # For now, I will choose the order exclusively by manhattan length, but future implementations might look at congestion as well.
    # One option is to compute length for each net and then sort a list of length,net_name tuples by their length, but this would be O(n) + O(nlogn).
    # This will almost certainly not be the bottleneck for this algorithm, but we might as well try to make things efficient from the start. A better 
    # way will be utilizing the fact that the max lenght of a net is not that big, even for the largest netlists. We can create buckets (list of lists),
    # one for each possible net length, and then appending cell names to their corresponding bucket. This is faster because the max possible manhattan length
    # is going to be 3000, since the largest grid we have is 1500x1500. 
    # How routing order is determined will be revisited later depending on how many ripups and failures that we get. 

    max_dist = 2 * netlist['grid_size']                      
    buckets = [[] for _ in range(max_dist)]                  # initializes list [[], [], [], ..., []] of size max_dist
    for net_name, data in netlist['nets'].items():
        idx = (max_dist-1) - dist(*data['pins'])             # subtact the distance between the two pins in data['pins'] from the largest possible index to ensure correct ordering
        buckets[idx].append(net_name)        
    
    # buckets now has the form [[],[],[NET_1],[NET_2,NET_3], ..., [NET_100]] or something along those lines. Now I want to simply flatten the list and return the result.
    flattened_buckets = [subarray_element for subarray in buckets for subarray_element in subarray]         # now has the form list[str] instead of list[list[str]]

    return flattened_buckets

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