import time
import heapq
import math
from bitarray import bitarray
from copy import deepcopy
from collections import deque

#from Rtest.Rtest_100_100 import data
from Reval.Reval_100_400 import data

#pasting an example netlist for testing purposes. Will delete later. 
#data = {   'grid_size': 100,
#    'nets': {   'NET_0': {   'length': 3,
#                             'pins': [(99, 56), (99, 59)],
#                             'type': 'LOCAL'},
#                'NET_1': {   'length': 48,
#                             'pins': [(5, 72), (44, 63)],
#                             'type': 'MEDIUM'},
#                'NET_2': {   'length': 52,
#                             'pins': [(43, 74), (5, 60)],
#                             'type': 'LONG'}}}

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
            - incremental move cost, which starts with a base cost of 1 and increments with vias and penalties due to congestion. 

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

        # we do the same thing, but this array keeps track of occpancy for pins. We need all pin positions on m2 to be occupied before routing, but we dont want to occupy those positions in self.occ because A_star_detailed() would complain that endpoints are occupied. It is easiest to create a second occupancy array for pins alone. 
        self.pin_occ = bitarray(N)
        self.pin_occ.setall(0)

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

    def is_pin(self, x:int, y:int, layer:int) -> bool:
        """ Returns True if x, y, layer is both in-bounds and is the position of an already placed pin."""
        if not self.in_bounds(x,y,layer):
            return False
        return bool(self.pin_occ[self.coordinate_to_idx(x,y,layer)])

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

def A_star_global(
        start,
        goal,
        routing_db,
        endpoints_are_tiles: bool = False,
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
    if endpoints_are_tiles:
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
        cost += congestion_weight * routing_db.tile_cong[nx][ny]    # added cost for entering congested tile

        move_dir = (nx - x, ny - y)                                 # move direction determined by a tuple. (-1, 0) means a leftward move, (0,1) means an upward move, etc.
        if (prev_dir != (0,0)) and (move_dir != prev_dir):          # move direction is compared against the previous move's direction. If they are not alike, then a bend has occurred. (0,0) included for start tile. 
            cost += turn_penalty

        return cost, move_dir

    g = {start_tile: 0.0}                                            # initializing the g dictionary. Each cell other than the start will be initialized to infinity 

    open_set = []
    heapq.heappush(open_set, (h(start_tile, goal_tile), start_tile)) # pushing the tuple (f_start,start) to the heap open_set. the first element of each tuple determines the order in which elements are popped. 

    # Initialize an unordered set of nodes already expanded (popped and processed). With a consistent heuristic, once a node is expanded its best g is final, so we never expand it again. This is needed if we allow duplicates in open_set (we do)
    closed = set()                  

    came_from = {start_tile: None}       # initializes the came_from dict that allows us to retrace steps. key: value tells us that to get to key, we came from value. 
    dir_from = {start_tile: (0,0)}       # initializes dir_frmo dict that stores the direction used to arrive at each tile. See step_cost for how the tuple corresponds to direction

    while open_set:
        current = heapq.heappop(open_set)[1]        # popping the (f,tile_coord) tuple from the top of the heap. Discarding the f value used for heap ordering. 

        # skipping stale heap entries; with duplicates allowed, we need to skip cells that have already been expanded. We never need to re-expand as long as we have a consistent heuristic (its g value is already optimal)
        if current in closed:
            continue

        # if current is the goal, then we are done. Reconstruct the path. 
        if current == goal_tile:
            return reconstruct_path(came_from, current)
        
        closed.add(current)     # by popping current from open_set, we are visiting the node. So we add it to closed.

        for neighbor in find_neighbors(current):
            prev_dir = dir_from[current]                                    # keeping track of the direction from which we arrived at current for detecting direction changes
            move_cost, move_dir = step_cost(current, neighbor, prev_dir)    # identifying cost and direction for stepping into neighbor from current      
            tentative_g = g[current] + move_cost                            # g value for stepping into the neighbor is the g to reach current plus the cost associated with the step into neighbor

            if tentative_g < g.get(neighbor, math.inf):                     # this is where the "each cell other than the start is initialized to infinity" comes into play. if neighbor does not yet exist in the g dict, then we use infinity as a placeholder.
                came_from[neighbor] = current                              
                dir_from[neighbor] = move_dir                               # direction to arrive at neighbor stored in dir_from
                g[neighbor] = tentative_g                                   

                f_neighbor = tentative_g + h(neighbor, goal_tile)           # total f value for the cell is used for ordering in the heap
                heapq.heappush(open_set, (f_neighbor, neighbor))            # pushing the neighbor into the frontier. Duplicates will occur, but old values of neighbor that are already in open_set with larger f[neighbor] values will never be visited due to the "if current in closed" check at the start of the loop

    print(f"A* terminated with no path found between {start_tile} and {goal_tile}.")
    return None

def build_corridor_bands(corridor_tiles, num_tiles, max_band=3):
    """"
    Returns a list of sets. bands[i] is a set of tiles at manhattan distance i from the corridor, where 0 <= i <= max_band.
    Tiles beyond max_band are not included.
    """
    bands = [set() for _ in range(max_band+1)]
    dist = [[None for _ in range(num_tiles)] for _ in range(num_tiles)]      # creates 2D grid matching the dimensions of the global placement grid; all entries initialized to None

    q = deque()     # creating a deque (double ended queue). We use a deque for fast popping on the left and appending on the right.
    for (tx, ty) in corridor_tiles:
        if 0 <= tx < num_tiles and 0 <= ty < num_tiles:
            dist[tx][ty] = 0            # all corridor tiles are a distance of 0 away from the corridor tiles (obviously)
            q.append((tx,ty))   
            bands[0].add((tx,ty))       # all cooridor tile coords are added to the set of cells a distance of 0 away from the corridor tiles
    
    # we are going to loop through q now. We start with the cells on the corridor, and we go in order of distance away from the corridor
    while q:
        tx, ty = q.popleft()            # returns and removes the element on the left end of the deque. Therefore, we start with the first element we added to q. 
        d = dist[tx][ty]                # returns the value of the distance of tile (tx,ty) from the corridor. We start with tiles on the corridor, so d is 0 for the starting tiles
        if d >= max_band:               # skip over tiles that are too far from the cooridor
            continue

        for (nx,ny) in [(tx+1, ty), (tx-1, ty), (tx, ty+1), (tx, ty-1)]:             # loop thru all neighbors of tile (tx, ty)
            if 0 <= nx < num_tiles and 0 <= ny < num_tiles and dist[nx][ny] is None: # checks that (nx,ny) is in-bounds and that we haven't already looked at this tile 
                dist[nx][ny] = d + 1                                                 # all neighbors *that we haven't already looked at* must be a further 1 tile away from the corridor
                if d + 1 <= max_band:
                    bands[d+1].add((nx,ny))                                          # if this tile is not too far away, then we add it to the corresponding bands set
                q.append((nx,ny))                                                    # adding this cell to the end of q

    return bands

def A_star_detailed(start, goal, routing_db, global_route, num_layers) -> list[tuple[int,int,int]]:
    """
    Uses A* search on the detailed routing grid to find a detailed route between start and goal.
    Even layers (0, 2, 4, 6) allow only vertical moves;
    Odd layers (1, 3, 5, 7) allow only horizontal moves;
    Via moves between adjacent layers at the same (x,y) position. 

    This function assumes routing_db layers correspond to M2 thru M9, where layer=0 means M2
    and layer=7 means M9. Start/goal pins from the netlist are provided as (x,y) pairs, but they 
    are treated as (x,y,0). 

    Cost breakdown is as follows: 
        wire steps have base cost of 1,
        vias have cost of 2 for adjacent-layer transitions,
        optional congestion penalty (routing_db.congestion_penalty),
        penalty for leaving global route corridor (if global route is provided).

    Returns path: list[(x,y,layer)] from start to goal inclusive, or None if unroutable
        
    """
    # raise an error if mismatch in layer num
    if routing_db.num_layers != num_layers:
        raise ValueError("Number of layers must match with database")
    
    # raise an error if coordinates are missing `layer`
    if len(start) != 3 or len(goal) != 3:
        raise ValueError("Bad coordinate format. Ensure that start and goal have dimensions (x, y, layer)")
    
    # return empty route if start or goal are not in-bounds (should never happen)
    if not routing_db.in_bounds(*start) or not routing_db.in_bounds(*goal):
        print(f"Detailed A*: start/goal out of bounds: start={start}, goal={goal}")
        return None
    
    # return empty route if start or goal are already occupied by wires (not pins)
    if routing_db.occ[routing_db.coordinate_to_idx(*start)] or routing_db.occ[routing_db.coordinate_to_idx(*goal)]:
        print(f"Detailed A*: start/goal occupied by previously committed wire: start={start}, goal={goal} ")
        return None
    

    # implementing corridor penalty machinery if global routes is present
    corridor_tiles = set(global_route) if global_route is not None else None
    band_cost = [0.0, 2.0, 6.0, 12.0]       # band_cost[n] is the cost associated with moving into a tile a distance of n away from the corridor. This penalizes routes that exit the global route. Increasing these values further punishes this behavior.
    max_band = len(band_cost) - 1           # the number of bands passed to build_corridor_bands depends on the num of elements in band_cost
    bands = None                            # initializes bands with None. Will stay None if global routing did not occur. 
    if corridor_tiles is not None:
        bands = build_corridor_bands(corridor_tiles, routing_db.num_tiles, max_band=max_band)  

    def corridor_penalty_for_tile(tx, ty):
        """ Computes the penalty for stepping into the tile tx, ty"""
        if bands is None:
            return 0.0
        t = (tx, ty)
        for d in range(max_band+1):         # goal is to find what band t belongs to
            if t in bands[d]:               # recall that bands[d] is a set for fast searching
                return band_cost[d]     
        
        return band_cost[max_band] + 5.0    # constant penalty for all tiles beyond max_band. We should increase the additive penalty here if we want to further discourage further jumps

    def h(current, goal):
        """ Combines Manhattan distance with a lower-bound on via cost to traverse from current layer to the goal layer"""
        x, y, layer = current
        gx, gy, glayer = goal

        manhattan_dist = abs(y-gy) + abs(x-gx)        # one part of the heuristic is the manhattan distance between the projections of current and goal coordinates on a 2D grid
        via_to_goal_lb  = abs(layer - glayer)         # lower bound for number of vias needed is the number of layers separating the two coordinates. 

        # Additional vias will be required if dx and dy are nonzero AND glayer == layer. This is because we would need to switch layers to move in one of the directions to close the gap. However, I am unsure if we can add these extra vias to the lower bound without violating consistency. For that reason, I am going to only add lower bound for vias needed to traverse the layers between current and goal. 
    
        return manhattan_dist + 2.0*via_to_goal_lb
    
    def reconstruct_path(came_from, current):
        path = [current]
        while came_from[current] is not None:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def cell_allowed(nx, ny, nl):
        """Determines if a cell is allowed by peforming boundary checks, checking route occupancy array, and checking pin ocupancy array """
        if not routing_db.in_bounds(nx, ny, nl):
            return False
        idx = routing_db.coordinate_to_idx(nx, ny, nl)

        # check occupancy array for non-pins
        if routing_db.occ[idx]:
            return False

        # checking occupancy array for pins. If it is in the pin array, check if it is one of the current net's pins
        if routing_db.pin_occ[idx] and (nx, ny, nl) not in {start, goal}:
            return False

        return True

    def find_neighbors(node):
        x, y, layer = node
        if layer % 2 == 0:
            candidates = [(x, y+1, layer), (x, y-1, layer), (x, y, layer+1), (x, y, layer-1)]     # only vertical moves or vias on even layers
        else: 
            candidates = [(x+1, y, layer), (x-1, y, layer), (x, y, layer+1), (x, y, layer-1)]     # only horizontal moves or vias on odd layers
            
        return [cell for cell in candidates if cell_allowed(*cell)]             # candidates must be in bounds and unoccupied to be a valid neighbor. 
    
    def step_cost(current, neighbor):
        x, y, layer = current
        nx, ny, nlayer = neighbor
        base_cost = routing_db.step_cost(x, y, layer, nx, ny, nlayer)       # computes the base cost of the move, which consists of 1.0 + 2*(number of layers traversed) + congestion penalty (for entering a congested region)

        if global_route is None:
            return base_cost                # no need to compute penalties associated with leaving the global route if the global route does not exist. 

        tx = nx // routing_db.tile_size
        ty = ny // routing_db.tile_size

        return base_cost + corridor_penalty_for_tile(tx, ty)

    # Below is the main A_star loop. It is identical to the A_star loop in the global routing case, except this time direction checking does not make up part of the move cost. Everything else is the same, so see comments in A_star_global for additional details. 
    g = {start: 0.0}

    open_set = []
    heapq.heappush(open_set, (h(start, goal), start))

    closed = set()
    came_from = {start: None}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current in closed:
            continue

        if current == goal:
            return reconstruct_path(came_from, current)

        closed.add(current)

        for neighbor in find_neighbors(current):
            tentative_g = g[current] + step_cost(current, neighbor)         # same as in global routing, but without direction checking

            if tentative_g < g.get(neighbor, math.inf):
                came_from[neighbor] = current
                g[neighbor] = tentative_g
                f_neighbor = tentative_g + h(neighbor, goal)
                heapq.heappush(open_set, (f_neighbor, neighbor))

    print(f"Detailed A*: no path found between {start} and {goal}.")
    return None

"""
========================================================================================
BEGIN MAIN
========================================================================================
"""


netlist = deepcopy(data)

routing_order: list[str] = create_routing_order(netlist)       # creates a routing order in the form of a list of net names

# instantiate the routing database
routing_db = RoutingDB(
                grid_size=netlist['grid_size'], 
                num_layers=8,
                tile_size=10)

# placing all pins in the netlist before routing takes place
for net_name, net in netlist["nets"].items():
    for (x, y) in net["pins"]:
        idx = routing_db.coordinate_to_idx(x, y, 0)
        routing_db.pin_occ[idx] = 1

for net_name in routing_order:
    start_coord = netlist['nets'][net_name]['pins'][0]  # start_coord is the first pin in the 'pins' list
    goal_coord  = netlist['nets'][net_name]['pins'][1]  # goal_coord is the second pin in the 'pins' list

    # if the net is long, we start with global routing
    if netlist['nets'][net_name]['type'] == 'LONG': 
        global_route = A_star_global(
                            start               = start_coord,
                            goal                = goal_coord,
                            routing_db          = routing_db,
                            endpoints_are_tiles = False,
                            congestion_weight   = 1,
                            turn_penalty        = 0
                            )
    else:
        global_route = None     # detailed routing A_star takes global_route as a parameter, so if global routing does not occur, then we set the route to None

    # begin detailed routing
    detailed_route = A_star_detailed(
                        start         = start_coord,
                        goal          = goal_coord,
                        routing_db    = routing_db,
                        global_route  = global_route        
                        )
    
    routing_db.commit_route(net_name, detailed_route)       # committing the detailed route to the database, which automatically updates congestion

print(routing_db.net_routes)               # prints the routes that were added in detailed routing
