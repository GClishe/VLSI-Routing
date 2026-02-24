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
    Routing database for detailed/global routing bookkeeping.

    The responsibilities of this database are as follows:
        - Tracks detailed grid occupancy per (x, y, layer) using a 1D bitarray (each layer stored sequentially and in row-major order)
        - Stores each committed net route so that they can be ripped up later if necessary
        - Tracks a sense of global-tile congestion by dividing the detailed grid into tiles and keeping track of how many committed routes pass through each tile
        - Some helper functions/utilities, such as: 
            - coordinate/index transformation for conversions bewteeen (x, y, layer) and 1D occupancy grid index
            - bounds checks
            - checking if a particular cell is occupied
            - incremental move cost, which starts with a base cost of 1 and increments with vias and penalties due to congestion. May also later include added costs for bends and for leaving global-routing corridor

    This class does not enforce layer-direction legality (yet). It is more likely that this will be enforced in neighbor generation within A*.
    
    """
    def __init__(self, grid_size: int, num_layers: int, tile_size: int = 10):
        """
        Parameters:

        grid_size: int
            Detailed routing grid width & height. Grid is a square        
        num_layers: int
            Number of routing layers reperesented in the database. M1 is likely not included here since M1 is not available for routing.
        tile_size: int
            Global routing tile dimension in terms of detailed-grid cells. Used for congestion bookkeeping and global-route guidance. 
        """
        self.grid_size  = grid_size   
        self.num_layers = num_layers  
        self.tile_size  = tile_size   

        # Conceptually, we want to have a 3D grid, each cell accessed by occ[layer][x][y], but this is memory-heavy. Instead, we flatten (layer,x,y) into a single index on a bitarray. 
        N = num_layers * self.grid_size * self.grid_size          # N computes the total number of bits we need to store.
        self.occ = bitarray(N)                                    # creates a bitarray of size N
        self.occ.setall(0)                                        # initializes all bits to 0

        # Number of tiles per side for global grid. Ceil allows for edges to be assigned to tile even if tile_size does not divide grid_size cleanly. Those edge tiles are effectively smaller/clipped by boundary checks. 
        self.num_tiles = math.ceil(grid_size/tile_size)

        self.tile_cong = [[0 for _ in range(self.num_tiles)] for _ in range(self.num_tiles)] # 2D array tracking tile congestion. tile_cong[tx][ty] reports how many committed nets pass through the tile (tx,ty)

        self.net_routes = {}                                                                 # dict that contains net_name -> list of bitarray indices for cells on that net
              
    def coordinate_to_idx(self, x: int, y: int, layer: int) -> int:
        """ Convert (x,y,layer) to 1D occupancy array index """
        return (layer * self.grid_size + x) * self.grid_size + y 
    
    def idx_to_coordinate(self, idx: int) -> tuple[int, int, int]:
        """ Convert 1D occupancy array index to (x,y,layer) coordinate """
        layer = idx // (self.grid_size * self.grid_size)
        rem = idx % (self.grid_size * self.grid_size)
        x = rem // self.grid_size
        y = rem % self.grid_size
        return (x, y, layer)

    def in_bounds(self, x: int, y: int, layer: int) -> bool:
        """ Return true if (x, y, layer) lies within the tracked grid/layer ranges"""
        return (0 <= x <= self.grid_size-1) and (0 <= y <= self.grid_size-1) and (0 <= layer < self.num_layers)

    def get_tile(self, x: int, y: int, layer: int) -> tuple[int,int]:
        """
        Return (tx,ty) coordinate for global tile containing the (x,y) detailed grid coordinate.
        `layer` is accepted for convenience/validation, but tiling only requires (x,y).
        """
        if not (self.in_bounds(x,y,layer)):
            raise ValueError("Coordinate given is not in-bounds.")
        
        return (x // self.tile_size, y // self.tile_size)

    def is_free(self, x: int, y: int, layer: int) -> bool:
        """
        Return True if the (x,y,layer) location is in-bounds and unoccupied. Note that this is a purely blockage/occupancy
        check. Directionality constraints imposed by metal layer (only horizontal/vertical wires allowed) should be handled
        during neighbor generation in A*, not here. 
        """
        if not self.in_bounds(x,y,layer):
            return False
        idx = self.coordinate_to_idx(x,y,layer)             # converts (x,y,layer) coordinate to a 1D index 
        return (not self.occ[idx])                          # self.occ(idx) checks if the coordinate is occupied. If it is, the cell is not free. If it is not, the cell is free. Also performs bounds checks.

    # TODO complete this function
    def via_allowed(self, x: int, y: int, layer: int) -> bool:
        """
        Returns true if a via can be placed from `layer` to `layer+1` at (x,y).

        Will check if both endpoints are in-bounds, if both endpoint cells are free, if layer+1 exists
        """
        pass

    def tiles_on_net(self, net_name: str) -> list[tuple[int,int]]:
        """ 
        Return the list of (tx, ty) tiles touched by the committed route for `net_name`, in order of occurrence along the stored path
        """
        cell_indices = self.net_routes.get(net_name)
        if not cell_indices:
            return []

        tiles = []
        seen = set()

        for cell_idx in cell_indices:
            x, y, layer = self.idx_to_coordinate(cell_idx)  # obtains (x,y,layer) coordinate associated with cell_idx and unpacks it into respective variables
            tx, ty = self.get_tile(x, y, layer)             # those variables get passed into get_tile, which returns the tile containing that coordinate

            tile = (tx, ty)
            if tile not in seen:                            # if we havent already included this tile in our list,
                seen.add(tile)                              # add it to the list of tiles we have seen
                tiles.append(tile)                          # and add it to the tiles list

        return tiles

    def congestion_penalty(self, x: int, y: int) -> float:
        """
        Return an additive penalty for steping into (x,y) based on congestion.
        penalty may be computed as alpha * tile_cong[tx][ty] or some kind of combination between that and a local density measure.
        It should be kept cheap as it will be called many times.

        Currently, no penalty for entering congested regions. 
        """
        return 0.0

    def commit_route(self, net_name: str, path: list[tuple[int, int, int]]):
        """
        Commit a routed detailed path to the database.

        parameters:
        net_name:
            Net identifier. Must not alerady be committed.
        path:
            List of (x,y,layer) points along the route, including endpoints.

        In-bounds checks are performed on every point, occupancy bits are set for points on the path, rejecting collisions if 
        the coordinate is already occupied, route is stored so that it can be ripped up later, and tile congestion is incremented once per unique 
        tile touched by this path. 
        """
        if net_name in self.net_routes:
            raise KeyError(f"{net_name} already committed")
        
        # validity checks
        path_indices: list[int] = []
        for (x, y, layer) in path:
            if not self.in_bounds(x, y, layer):
                raise ValueError(f"Coordinate out of bounds: {(x, y, layer)}")

            idx = self.coordinate_to_idx(x, y, layer)
            if self.occ[idx]:
                raise ValueError(f"collision at {(x, y, layer)} (idx={idx})")
            path_indices.append(idx)

        # commit occupancy bits
        for idx in path_indices:
            self.occ[idx] = 1

        # store route
        self.net_routes[net_name] = path_indices

        # increment congestion in tile_cong
        tile_set = set()
        for idx in path_indices:
            x, y, layer = self.idx_to_coordinate(idx)
            tile = (x // self.tile_size, y // self.tile_size)
            if tile not in tile_set:
                tile_set.add(tile)
                tx, ty = tile
                self.tile_cong[tx][ty] += 1

        return None                               # not necessary, but return None helps with readability
    
    def rip_up(self, net_name: str):
        """
        Remove a previously committed route.
        
        Occupancy bits are cleared for the net's stored path, tile congestion is decremented by one for each tile the net touched, and the stored route entry is deleted. 
        """
        # clearing occupancy
        indices = self.net_routes[net_name]            
        for idx in indices:                 
            self.occ[idx] = 0                           
        
        affected_tiles = self.tiles_on_net(net_name)    # gets the tiles that this net passed through

        # decrementing tile congestion
        for (tx, ty) in affected_tiles:         
            if self.tile_cong[tx][ty] == 0:
                raise ValueError("Tile congestion value cannot be negative")
            else:
                self.tile_cong[tx][ty] -= 1
        
        # remove route from the database
        self.net_routes.pop(net_name)

    def step_cost(self, x: int, y: int, layer: int, nx: int, ny: int, nlayer: int) -> float:
        """
        Incremental cost to move from (x,y,layer) to (nx,ny,nlayer) for usage inside A*. 
        Wire step cost: 1 per grid move.
        Via cost: 2 per adjacent-layer transition. Stacked vias are allowed.
        Congestion penalty: computed above.
        """

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

def A_star(
        start,
        goal,
        routing_db,
        endboints_are_tiles: bool = False,
        congestion_weight: float = 1.0,
        turn_penalty: float = 0.0
):
    """
    A* search on global tile grid. This is for the tile grid only (global routing). Detailed
    routing will use this tile path to form a preferred corridor for detailed routes. 

    parameters: 
    start, goal: 
        If endpoints_are_tiles is False (default):
            Start and goal are detailed-grid coordinates (x,y,layer).
            These are mapped to tile coords via routing_db.get_tile().
        If endpoints_are_tiles is True:
            start and goal are already tile coords (tx,ty).
    routing_db:
        Routing_DB instance.
    congestion_weight:
        Multiplier on tile congestion when computing step cost. 
        Step cost to enter tile (nx,ny) is 1 + congestion_weight * tile_cong[nx][ny].
        Useful for considering tile congestion in global routing. 
    turn_penalty:
        Optional extra cost if direction changes between successive moves. Each time 
        we change direction in the global route, it is kind of like a proxy for via
        cost, since changing direction in a detailed route has a cost due to directionality
        rules on each layer. turn_penalty encourages straighter global routes. Defaulted to 0. 

    Returns:
    path_tiles:
        list of (tx,ty) tiles from start to goal inclusive, or None if no path is found.
    
    """
    # first, we need to map the detailed coordinate start and goal to tile coordinates.
    if endboints_are_tiles:
        start_tile = start
        goal_tile = goal
    else: 
        sx, sy, sl = start      # start_x, start_y, start_layer
        gx, gy, gl = goal       # same for goal
        start_tile = routing_db.get_tile(sx, sy, sl)  
        goal_tile  = routing_db.get_tile(gx, gy, gl)

    num_tiles = routing_db.num_tiles    # this is number of tiles to a size on the global tile grid. A square grid (which we have) has num_tiles^2 total tiles. 

    def tile_in_bounds(tile):
        """ Decides if a tile is in-bounds"""
        tx, ty = tile
        return (0 <= tx < num_tiles) and (0 <= ty < num_tiles)
    
    def h(current, goal):
        """
        Currently using manhattan distance as the heuristic. This heuristic is consistent. Our step cost is 1 + nonnegative 
        congestion + nonnegative turn penalty, so the cost for each step is always at least one. Since Manhattan distance 
        decreases by at most 1 per step, we have h(n,goal) <= c(n,n') + h(n',goal) for all neighbors n' of n, which is 
        the condition that must be met for a consistent heuristic.
        """
        return dist(current, goal)

    def reconstruct_path(came_from, current):
        """
        Traces back the path from current_node back to the start of move_list. Successive indices in the output list indicate successive nodes in the path. 
        """
        path = [current]                            # start at the goal node (this function is only called when current == goal)
        while came_from[current] is not None:       # came_from is the list of key: value pairs where in order to get to key, we traversed through value. came_from[start] == None
            current = came_from[current]            # moving down the list toward the start
            path.append(current)                    
        path.reverse()                              # reverse the list to get a start --> goal path instead of a goal --> start path. 
        return path

    def find_neighbors(current_cell):
        """
        Returns the in-bounds tiles that are neighbors of current_cell. This is much simpler for global routing because only one layer need be considered.
        """
        tx, ty = current_cell
        candidates = [(tx + 1, ty), (tx - 1, ty), (tx, ty + 1), (tx, ty - 1)]   # list of candidate tiles
        return [tile for tile in candidates if tile_in_bounds(tile)]            # returns the candidates that are in-bounds

    def step_cost(current, neighbor, prev_dir):
        """
        Returns the cost to step frmo current tile to the neighbor tile and the direction of that move.
        Cost starts at 1 for a tile step. Adds congestion_weight*tile_cong[neighbor] and 
        also a turn penalty if direction changes relative to prev_dir.

        """
        x, y   = current
        nx, ny = neighbor
        cost = 1.0                                                  # base cost for stepping one tile
        cost += congestion_weight * routing_db.tile_cong[nx,ny]     # added cost for entering congested tile

        move_dir = (nx - x, ny - y)                                 # move direction determined by a tuple. (-1, 0) means a leftward move, (0,1) means an upward move, etc.
        if (prev_dir != (0,0)) and (move_dir != prev_dir):          # move direction is compared against the previous move's direction. If they are not alike, then a bend has occurred.
            cost += turn_penalty

        return cost, move_dir





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