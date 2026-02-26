import time
import heapq
import math
from bitarray import bitarray
from copy import deepcopy
from collections import deque
from dataclasses import dataclass, field

from Rtest.Rtest_500_6000 import data
#from Reval.Reval_1000_30000 import data

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


@dataclass      # dataclass decorator useful for classes like these whose entire purpose is holding data. It simply automatically generates boilerplate methods like __init__ so that we do not have to. 
class RouterParams:
    """
    Throughout the routing procedure, there are various tuning knobs in many different places.
    This RouterParams class is built to centralize all of these tuning knobs into one place.
    These tuning knobs are passed as parameters into the RoutingDB class so that they need not 
    be modified there. 
    """
    # geometry parameters
    num_layers: int = 8
    tile_size: int = 10

    # cost model for wire steps and via steps. These should remain unchanged.
    wire_step_cost: float = 1.0
    via_cost: float = 2.0

    # congestion penalty defined as alpha * util^p where util is the utilization percentage of the tile (number of occupied cells in the tile divided by total number of cells in the tile)
    cong_alpha: float = 8.0
    cong_p: float = 2.0

    # global routing params
    do_global_for_types: tuple[str, ...] = ("LONG","MEDIUM")   # alternative might be ("MEDIUM","LONG") if global routing is to be done on medium type nets
    global_cong_weight: float = 0.002                  # global routing step cost defined as 1 + congestion_weight * tile_cong. tile_cong counts how many occupied cells are present in the tile, which may be large. therefore, global_cong_weight should be small
    global_turn_penalty: float = 0.5                   # penalty for changing direction in global routing

    # penalties associated with leaving global routing corridor while doing detailed routing
    # field is used for corridor_band_cost because it is mutable; we need to ensure that different instances of the RouterParams class contain separate lists if we only wish to mutate corridor_band_cost for only one of the instances. It is not too necessary here, but it is good practice. 
    corridor_band_cost: list[float] = field(default_factory=lambda: [0.0, 1.0, 3.0, 6.0])       # additional cost for stepping into tile bands a distance of idx away from the global route corridor
    corridor_far_penalty: float = 3.0                  # penalty for entering cells very far from global route corridor is corridor_band_cost[-1] + corridor_far_penalty
    corridor_util_thresh: float = 0.02                 # only penalize leaving corridor if util >= thresh

    # params for retrying
    max_reroute_attempts: int = 3                      # how many times rerouting will occur
    ripup_k: int = 10                                  # how many previously-routed nets to rip up on failure
    relax_corridor_on_retry: bool = True               # disable corridor on retries
    bump_cong_alpha_on_retry: float = 1.5              # multiply cong_alpha by this each retry. Determines how much congestion penalty increases each retry
    max_requeue_per_net: int = 20                      # number of times a net can be placed back into the queue after being ripped up. Prevents infinite ripup - reroute loops

    # per-net routing time limit (seconds). If a single net takes longer than this amount across all
    # reroute attempts, it will be marked as a trouble net and treated specially to avoid hanging.
    max_time_per_net: float = 60.0

    hist_weight: float = 2.0         # multiplier on history term in congestion penalty
    hist_inc: float = 1.0            # how much to increment history when tile is over threshold
    hist_util_thresh: float = 0.10   # only build history in tiles above this utilization

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
    def __init__(self, grid_size: int, params: RouterParams):
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
        self.num_layers = params.num_layers
        self.tile_size  = params.tile_size
        self.params     = params

        # Conceptually, we want to have a 3D grid, each cell accessed by occ[layer][x][y], but this is memory-heavy. Instead, we flatten (layer,x,y) into a single index on a bitarray. 
        N = self.num_layers * self.grid_size * self.grid_size          # N computes the total number of bits we need to store.
        self.occ = bitarray(N)                                    # creates a bitarray of size N
        self.occ.setall(0)                                        # initializes all bits to 0

        # we do the same thing, but this array keeps track of occpancy for pins. We need all pin positions on m2 to be occupied before routing, but we dont want to occupy those positions in self.occ because A_star_detailed() would complain that endpoints are occupied. It is easiest to create a second occupancy array for pins alone. 
        self.pin_occ = bitarray(N)
        self.pin_occ.setall(0)

        # Number of tiles per side for global grid. Ceil allows for edges to be assigned to tile even if tile_size does not divide grid_size cleanly. Those edge tiles are effectively smaller/clipped by boundary checks. 
        self.num_tiles = math.ceil(grid_size/self.tile_size)

        self.tile_cong = [[0 for _ in range(self.num_tiles)] for _ in range(self.num_tiles)] # 2D array tracking tile congestion. tile_cong[tx][ty] reports how many committed nets pass through the tile (tx,ty)
        self.tile_hist = [[0.0 for _ in range(self.num_tiles)] for _ in range(self.num_tiles)]  # creates tile history term

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
        Return a penalty for entering congested tiles. After ripup, utilization drops and so congestion penalty would ordinarily disappear.
        However, we also add a history penalty that allows the router to 'remember' that the tile was problematic beforehand
        """
        tx = x // self.tile_size
        ty = y // self.tile_size

        d = self.tile_cong[tx][ty]      # grabs the congestion value for tile (tx,ty) (number of committed cells in that tile)
        cap = self.tile_size * self.tile_size * self.num_layers         # creates a capacity value; determines how many cells can exist in (tx,ty)
        util = d / cap                                       # defines utilization of the tile by normalizing the congestion value with the capacity of the tile

        # congestion penalty is exponential; penalty = alpha * util^p. at low util, congestion penalty is near zero, grows faster as util rises
        # these values should be tuned
        a = self.params.cong_alpha
        p = self.params.cong_p
        base = a * (util ** p)
        hist = self.params.hist_weight * self.tile_hist[tx][ty]     # considers tile history in addition to its congestion

        return base + hist

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
        tile_counts = {}
        for idx in path_indices:                                        # looping through all the indices of cells that belong to the path
            x, y, layer = self.idx_to_coordinate(idx)                   # getting the coordinates of those cells
            tx, ty = (x // self.tile_size, y // self.tile_size)         # getting the tile coordinate for the cells
            tile_counts[(tx, ty)] = tile_counts.get((tx, ty), 0) + 1    # incrementing tile_counts, which counts how many committed cells belong to this tile as a measure of congestion

        for (tx, ty), cnt in tile_counts.items():
            self.tile_cong[tx][ty] += cnt                               # updating the self.tile_cong data with the congestion values

        return None                               # not necessary, but return None helps with readability
    
    def rip_up(self, net_name: str):
        """
        Remove a previously committed route.
        
        Occupancy bits are cleared for the net's stored path, tile congestion is decremented by 
        the number of occupied cells that this net contributed to each tile. 
        """
        # sanity check
        if net_name not in self.net_routes:
            raise KeyError(f"{net_name} is not committed")
    
        indices = self.net_routes[net_name]     # indices for the cells on this net

        # we need to recompute the contribution to tile congestion for `net_name` so that we can subtract it from the total congestion.
        tile_counts = {}            # dict containing (tx,ty):count items where count tells how many detailed cells on net_name belong to tx,ty
        for idx in indices:
            x, y, layer = self.idx_to_coordinate(idx)
            tx, ty = self.get_tile(x, y, layer)
            tile_counts[(tx,ty)] = tile_counts.get((tx,ty),0) + 1  # incrementing tile count for (tx,ty)

        # clearing occupancy
        for idx in indices:                 
            self.occ[idx] = 0                           
        
        affected_tiles = self.tiles_on_net(net_name)    # gets the tiles that this net passed through

        # decrementing tile congestion
        for (tx,ty), cnt in tile_counts.items():
            new_val = self.tile_cong[tx][ty] - cnt      # new congestion value is the total congestion in the tile minus the contribution from `net_name`, which is being ripped up
            if new_val < 0:
                raise ValueError(
                    f"Tile congestion would go negative at tile {(tx, ty)}: "
                    f"{self.tile_cong[tx][ty]} - {cnt}"
                )
            self.tile_cong[tx][ty] = new_val
        
        # remove route from the database
        self.net_routes.pop(net_name)

    def step_cost(self, x: int, y: int, layer: int, nx: int, ny: int, nlayer: int) -> float:
        """
        Incremental cost to move from (x,y,layer) to (nx,ny,nlayer) for usage inside A*. 
        Wire step cost: 1 per grid move.
        Via cost: 2 per adjacent-layer transition. Stacked vias are allowed.
        Congestion penalty: computed above.
        """
        cost = self.params.wire_step_cost           # base cost. each step has a cost of at least 1
        if layer != nlayer:
            cost += self.params.via_cost * abs(nlayer - layer)       # vias cost 2 for each layer traversal. Since we can stack vias, we need to compute how many layers are traversed. 

        cost += self.congestion_penalty(nx, ny)     # add a congestion penalty

        return cost
    
    def update_history_penalties(self):
        """
        If a tile's utilization exceeds hist_util_thresh, increment its history penalty.
        Goal of the history penalty is to make the router stop visiting the same overfull areas. 
        """
        cap = self.tile_size * self.tile_size * self.num_layers     # tile capacity
        thr = self.params.hist_util_thresh                          # history utilisation threshold
        inc = self.params.hist_inc                                  # increment penalty factor

        for tx in range(self.num_tiles):
            for ty in range(self.num_tiles):
                util = self.tile_cong[tx][ty] / cap
                if util > thr:
                    # increment proportional to how far above threshold we are
                    self.tile_hist[tx][ty] += inc * (util - thr)

def dist(c1, c2):
        #returns the manhattan distance between coordinates c1 and c2
        x1,y1 = c1     # unpacks current coordinates
        x2,y2 = c2     # unpacks goal coordinates
        return abs(x1-x2) + abs(y1-y2)

def create_routing_order(netlist: dict) -> list[str]:
    """
    Orders nets for routing with the following priority: 
    First priority is given to nets that require at least one turn (dx>0 AND dy=0).
    Within each of the two classes (turn required/not required), prioritize long manhattan distance. 
    """

    max_dist = 2 * netlist['grid_size']                      
    buckets_turn = [[] for _ in range(max_dist)]            # grouping for nets requiring a turn
    buckets_straight = [[] for _ in range(max_dist)]        # grouping for nets that dont require a turn

    for net_name, data in netlist['nets'].items():
        (x0, y0), (x1, y1) = data['pins']
        dx = abs(x0-x1)
        dy = abs(y0-y1)

        d = dx + dy
        idx = (max_dist-1) - d      # nets will be placed into the bucket corresponding to its length, where longest possible nets (length of max_dist) are placed in index 0; shortest lengths placed in the back.

        if dx != 0 and dy != 0:
            buckets_turn[idx].append(net_name)
        else:
            buckets_straight[idx].append(net_name)
        
    nets_with_turn = [n for bucket in buckets_turn for n in bucket]         # flattens the buckets_turn list
    nets_without_turn = [n for bucket in buckets_straight for n in bucket]  # flattens the buckets_straight list\

    return nets_with_turn + nets_without_turn          # nets with turn come first, ordered by length, then nets without turn are routed, ordered by length. 

def A_star_global(
        start,
        goal,
        routing_db,
        params: RouterParams,
        endpoints_are_tiles: bool = False
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
        cost += params.global_cong_weight * routing_db.tile_cong[nx][ny]    # added cost for entering congested tile

        move_dir = (nx - x, ny - y)                                 # move direction determined by a tuple. (-1, 0) means a leftward move, (0,1) means an upward move, etc.
        if (prev_dir != (0,0)) and (move_dir != prev_dir):          # move direction is compared against the previous move's direction. If they are not alike, then a bend has occurred. (0,0) included for start tile. 
            cost += params.global_turn_penalty

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

def build_allowed_tiles_from_bands(bands: list[set[tuple[int,int]]], band_limit: int) -> set[tuple[int,int]]:
    """Bands built by build_corridor_bands flattened into a single set of allowed tiles"""
    if bands is None:
        return None
    band_limit = min(band_limit, len(bands) - 1)
    allowed = set()
    for d in range(band_limit + 1):
        allowed |= bands[d] 
    return allowed

def A_star_detailed(start, goal, routing_db, global_route, params, band_limit) -> list[tuple[int,int,int]]:
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
    # raise an error if coordinates are missing `layer`
    if len(start) != 3 or len(goal) != 3:
        raise ValueError("Bad coordinate format. Ensure that start and goal have dimensions (x, y, layer)")
    
    # return empty route if start or goal are not in-bounds (should never happen)
    if not routing_db.in_bounds(*start) or not routing_db.in_bounds(*goal):
        print(f"Detailed A*: start/goal out of bounds: start={start}, goal={goal}")
        return None
    
    s_idx = routing_db.coordinate_to_idx(*start)        # DB index for start
    g_idx = routing_db.coordinate_to_idx(*goal)         # DB index for goal

    # return empty route if start or goal are already occupied by wires (not pins)
    if routing_db.occ[s_idx] or routing_db.occ[g_idx]:
        print(f"Detailed A*: start/goal occupied by previously committed wire: start={start}, goal={goal} ")
        return None
    
    sx, sy, sl = start
    gx, gy, gl = goal

    # implementing corridor penalty machinery if global routes is present
    corridor_tiles = set(global_route) if global_route is not None else None
    band_cost = params.corridor_band_cost   # band_cost[n] is the cost associated with moving into a tile a distance of n away from the corridor. This penalizes routes that exit the global route. Increasing these values further punishes this behavior.
    max_band = len(band_cost) - 1           # the number of bands passed to build_corridor_bands depends on the num of elements in band_cost
    bands = None                            # initializes bands with None. Will stay None if global routing did not occur. 
    allowed_tiles = None
    if corridor_tiles is not None:
        bands = build_corridor_bands(corridor_tiles, routing_db.num_tiles, max_band=max_band)  
        allowed_tiles = build_allowed_tiles_from_bands(bands, band_limit)


    def corridor_penalty_for_tile(tx, ty):
        """ Computes the penalty for stepping into the tile tx, ty"""
        if bands is None:
            return 0.0
        
        cap = (routing_db.tile_size * routing_db.tile_size) * routing_db.num_layers     # defines the capacity of a tile as its area times depth (in terms of detailed cells)
        util = routing_db.tile_cong[tx][ty] / cap                                       # computes utilization by dividing the congestion (number of committed cells in tile) by its capacity

        # exiting the global routing corridor will only be punished if the tile we want to step into is congested above the threshold.
        if util < params.corridor_util_thresh:
            return 0.0

        t = (tx, ty)
        for d in range(max_band+1):         # goal is to find what band t belongs to
            if t in bands[d]:               # recall that bands[d] is a set for fast searching
                return band_cost[d]     
        
        return band_cost[max_band] + params.corridor_far_penalty    # constant penalty for all tiles beyond max_band. We should increase the additive penalty here if we want to further discourage further jumps

    def h(node_idx):
        """ Combines Manhattan distance with a lower-bound on via cost to traverse from current layer to the goal layer"""
        x, y, layer = routing_db.idx_to_coordinate(node_idx)

        manhattan_dist = abs(y-gy) + abs(x-gx)    # one part of the heuristic is the manhattan distance between the projections of current and goal coordinates on a 2D grid
        via_to_goal_lb  = abs(layer - gl)         # lower bound for number of vias needed is the number of layers separating the two coordinates. 

        # Additional vias will be required if dx and dy are nonzero AND glayer == layer. This is because we would need to switch layers to move in one of the directions to close the gap. However, I am unsure if we can add these extra vias to the lower bound without violating consistency. For that reason, I am going to only add lower bound for vias needed to traverse the layers between current and goal. 
    
        return manhattan_dist + 2.0*via_to_goal_lb
    
    def reconstruct_path(came_from, end_idx):
        out_idx = [end_idx]     # initializing out_idx list
        cur = end_idx
        while True:
            prev = came_from.get(cur, -1)       # if cur doesnt exist in came_from, return -1
            if prev == -1:                      # and then break
                break
            out_idx.append(prev)                # adding the cell that cur came from to the list
            cur = prev                          # updating cur
        out_idx.reverse()                       # reversing the list to go from start to end instead of end to start
        return [routing_db.idx_to_coordinate(i) for i in out_idx]       # turning DB indices back into coordinates
    
    def cell_allowed(nx, ny, nl):
        """Determines if a cell is allowed by peforming boundary checks, checking route occupancy array, and checking pin ocupancy array """
        if not routing_db.in_bounds(nx, ny, nl):
            return False
        
        idx = routing_db.coordinate_to_idx(nx, ny, nl)

        # check occupancy array for non-pins
        if routing_db.occ[idx]:
            return False

        # checking occupancy array for pins. If it is in the pin array, check if it is one of the current net's pins
        if routing_db.pin_occ[idx] and idx not in [s_idx, g_idx]:
            return False
        
        if allowed_tiles is not None:
            tx, ty = routing_db.get_tile(nx, ny, nl)
            if (tx, ty) not in allowed_tiles:
                return False

        return True

    # Below is the main A_star loop. It is similar to the global routing A*, but this version works primarily with DB indices rather than coordinate values. 
    g = {s_idx: 0.0}
    came_from = {s_idx: -1}

    h_start = h(s_idx)
    f_start = h_start
    open_set = [(f_start, h_start, s_idx)]     # heap is slightly modified; h is used to break ties in f

    closed = set()
    
    while open_set:
        cur_idx = heapq.heappop(open_set)[2]    # extract only s_idx; indices 0 and 1 are used for heap ordering

        if cur_idx in closed:
            continue

        if cur_idx == g_idx:
            return reconstruct_path(came_from, cur_idx)

        closed.add(cur_idx)
        x, y, layer = routing_db.idx_to_coordinate(cur_idx)     # extracting the coordinate associated with cur_idx

        # neighbor candidates are assigned here. Even layer neighbors include only via steps and vertical steps; odd layer neighbors include only via steps and horizontal steps
        if layer % 2 == 0:
            candidates = [(x, y + 1, layer), (x, y - 1, layer), (x, y, layer + 1), (x, y, layer - 1)]
        else:
            candidates = [(x + 1, y, layer), (x - 1, y, layer), (x, y, layer + 1), (x, y, layer - 1)]

        cur_g = g[cur_idx]
        for nx, ny, nl in candidates:
            if not cell_allowed(nx, ny, nl):
                continue

            n_idx = routing_db.coordinate_to_idx(nx, ny, nl)

            base = routing_db.step_cost(x, y, layer, nx, ny, nl)            # base cost for stepping into nx, ny, nl. This includes H/V steps, via steps, and congestion penalties

            if corridor_tiles is not None:
                tx, ty = routing_db.get_tile(nx, ny, nl)                    # corridor tiles is none if global routing did not take place.
                base += corridor_penalty_for_tile(tx, ty)                   # adding corridor penalty to the cost if global routing did occure
            

            tentative_g = cur_g + base                                       # same as in global routing, but without direction checking
            if tentative_g < g.get(n_idx, math.inf):
                g[n_idx] = tentative_g
                came_from[n_idx] = cur_idx
                hn = h(n_idx)
                fn = tentative_g + hn
                heapq.heappush(open_set, (fn, hn, n_idx))

    print(f"Detailed A*: no path found between {start} and {goal}.")
    return None

def total_routing_cost_from_routes(routes: dict[str,list[tuple[int,int,int]]]):

    WIRE_COST = 1.0
    VIA_COST  = 2.0

    total = 0.0

    for net_name, path in routes.items():            # grabs the net name and the path for that net
        # skip unrouted nets 
        if not path or len(path) < 2:
            continue

        prev = path[0]                # to find the cost, we need to keep track of previous coordinate and current coordinate. previous coordinate initialized to the first one

        for (x1,y1,l1) in path[1:]:   # starting the loop at the second coordinate, since previous coordinate is initialized to the first          
            x0, y0, l0 = prev

            # finding the change in x, y, and layer
            dx = abs(x1 - x0)           
            dy = abs(y1 - y0)
            dl = abs(l1 - l0)

            wire_len = dx + dy      # should be 1 or 0
            via_len  = dl           # should be 1 or 0

            # double checking that each move is either a wire step (wire length is 1) or a via step (via length is 1)
            is_wire_step = (wire_len == 1) and (via_len == 0)
            is_via_step  = (wire_len == 0) and (via_len == 1)
            if not (is_wire_step or is_via_step):
                raise ValueError(
                    f"{net_name}: invalid step {(x0,y0,l0)} -> {(x1,y1,l1)} "
                    f"(dx={dx}, dy={dy}, dl={dl})"
                )

            total += WIRE_COST * wire_len
            total += VIA_COST  * via_len
            prev = (x1, y1, l1)             # updating prev

    return total

def choose_ripup_nets(routing_db: RoutingDB, routed_nets: list[str], start: tuple[int, int, int], goal: tuple[int,int,int], k: int) -> list[str]:
    """"
    Chooses nets for ripup, prioritizing nets with a large presence in the neighborhoods around start and goal. If few such nets
    exist, then we ripup the most recently routed nets. 
    """
    (sx, sy, sl) = start
    (gx, gy, gl) = goal
    st = routing_db.get_tile(sx, sy, sl)
    gt = routing_db.get_tile(gx, gy, gl)


    neigh = set()
    # add the neighbor tiles of st and gt to the neigh set. The loops below biuld a 3x3 box centered on each endpoint, clipped to grid boundaries
    for (tx,ty) in [st, gt]:     # look at the start and goal tiles
        for dx in [-1,0,1]:
            for dy in [-1,0,1]:
                nx, ny = tx + dx, ty + dy
                if 0 <= nx < routing_db.num_tiles and 0 <= ny < routing_db.num_tiles:
                    neigh.add((nx, ny))
    
    window = routed_order[-5 * k:] if len(routed_order) > 5 * k else routed_order[:]    # grabbing the most recent 5*k entries. If less than that many entries exist, then grab all of them
    recency_rank = {net: i for i, net in enumerate(window)}                             # ranking nets that increases with recency

    scored = []
    for net in window:                 
        tiles = set(routing_db.tiles_on_net(net))   # gets the tiles that the net passes through
        score = len(tiles & neigh)                  # compute the number of tiles that the net occupies in the neighborhood of the endpoints
        scored.append((score, recency_rank[net], net))                  # score each net by the number of neighbor cells they occupy

    scored.sort(key=lambda t: (t[0], t[1]), reverse=True)               # sorting scored with priority given to score; ties broken with recency rank

    picks = [net for (s, _, net) in scored if s > 0]       # extracts all nets that overlap with the start/goal neighborhood
    if len(picks) < k:                          # if picks is fewer than k, then we fill the remaining slots with most recent nets
        for net in reversed(routed_order):       # reverses the list of routed nets so that more recent nets come first 
            if net not in picks:                # add it to the list of picks if it's not already there
                picks.append(net)
            if len(picks) == k:                 # as soon as we get k nets, we break. 
                break
            
    return picks[:k] # picks may have more than k nets if there were lots of nets in start/goal neighbors, so we only grab the first k. 
        
def add_m1_vias_to_all_routes(routing_db, netlist):
    """
    Post-processing function that adds m1-m2 vias to start and end coordinates of each committed route on the database.
    This function does not modify routing_db.occ or routing_db.net_routes, which are m2-m9 only. 

    Returns routes_with_m1: dict[str, list[tuple[int,int,int]]], where each path is a list of (x,y,layer)
    where layer=-1 means M1, layer=0 means M2, ..., 7 means M9.
    """
    # create a new routes dictionary, but this time each route will have m2-m1 vias at the start and end
    routes_with_m1: dict[str, list[tuple[int, int, int]]] = {}

    for net_name, idx_path in routing_db.net_routes.items():        # loop thru all routes in the database
        # if the path is empty, skip it
        if not idx_path:        
            continue

        coords = [routing_db.idx_to_coordinate(idx) for idx in idx_path]   # getting (x,y,layer) coordinates for all cells on the path

        (x0, y0), (x1, y1) = netlist["nets"][net_name]["pins"]              # get the start/goal pins for the net

        # sanity checking that the first and last coordinates on the path (the pins) are on layer 0 (m2). We should be concerned if this check fails.
        if coords[0][2] != 0 or coords[-1][2] != 0:
            raise ValueError(
                f"{net_name}: expected endpoints on M2 (layer=0), got start={coords[0]} end={coords[-1]}"
            )

        # creating coordinates for the new start and end points, which are on layer -1 (m1)
        start_m1 = (coords[0][0], coords[0][1], -1)
        end_m1   = (coords[-1][0], coords[-1][1], -1)

        routes_with_m1[net_name] = [start_m1] + coords + [end_m1]       # adding the start and end points to the start and end of the list, respectively

    return routes_with_m1

def try_l_shape(start, goal, routing_db, layers=(0,1)):
    """
    Tries a simple L route; vertical on even layer, horizontal on odd layer, with vias at the bend. Returns path or None.
    Will be called before detailed A* is attempted.
    """
    (sx, sy, sl0) = start
    (gx, gy, gl0) = goal

    if sl0 != 0 or gl0 != 0:
        # pin positions must always be on m2. the layers parameter doesnt need to have m2 on it, but the pins must at least start there. 
        return None
    
    s_idx = routing_db.coordinate_to_idx(sx, sy, sl0)
    g_idx = routing_db.coordinate_to_idx(gx, gy, gl0)

    # vertical/horizontal layer depens on parity
    a, b = layers
    if (a % 2) == 0 and (b % 2) == 1:
        v_layer, h_layer = a, b
    elif (a % 2) == 1 and (b % 2) == 0:
        v_layer, h_layer = b, a
    else:
        # Need one even (vertical) and one odd (horizontal)
        return None

    def segment(a,b,layer):
        """Helper function that draws straight line paths between a and b if it is possible to do so. If not, it returns None."""
        # unpack a,b
        (x0, y0) = a
        (x1, y1) = b

        path = []
        if x0 == x1:
            # if x0 == x1, then we need a vertical wire
            step = 1 if y1 >= y0 else -1        # step upwards if y1 is greater than y0, else step down
            for y in range(y0, y1 + step, step):# on each iteration, we step closer. `step` field increments or decrements, depending on if y0 is smaller or larger than y1, respetively.
                # check if we are allowed to step toward y1 without running into something
                if not routing_db.is_free(x0, y, layer) and not routing_db.is_pin(x0, y, layer):
                    return None
                path.append((x0, y, layer))
        elif y0 == y1:
            # same as above. We incrementally step toward x1 from x0 and return None if we are blocked at any point along the way
            step = 1 if x1 >= x0 else -1
            for x in range(x0, x1 + step, step):
                if not routing_db.is_free(x, y0, layer) and not routing_db.is_pin(x, y0, layer):
                    return None
                path.append((x, y0, layer))
        else:
            # if neither x values or y values are the same, then we cannot make a straight segment between a and b. 
            return None
        return path
    
    def via_stack(x: int, y: int, l0: int, l1: int) -> list[tuple[int,int,int]]:
        """Inclusive via stack from l0 -> l1 in the order of traversal."""
        if l0 == l1:
            return [(x, y, l0)]
        step = 1 if l1 > l0 else -1
        return [(x, y, l) for l in range(l0, l1 + step, step)]
    
    def cell_ok(x: int, y: int, layer: int) -> bool:
        """Blocks wires and pins unless it's exactly this net's endpoints."""
        if not routing_db.in_bounds(x, y, layer):
            return False

        idx = routing_db.coordinate_to_idx(x, y, layer)

        # cannot collide with existing wires
        if routing_db.occ[idx]:
            return False

        # pins are blocked unless this coordinate is exactly start or goal
        if routing_db.pin_occ[idx] and idx not in (s_idx, g_idx):
            return False

        return True

    def stack_ok(stack: list[tuple[int, int, int]]) -> bool:
        """Helper function to validate an entire via stack because I have had problems with this."""
        return all(cell_ok(x, y, l) for (x, y, l) in stack)

    # we need to try creating bends in both the valid locations; (sx, gy) and (gx, sy). One might be blocked, so we try both before we say that we cant create an L route. 
    # option A: vertical (even) then horizontal (odd): bend at (sx, gy)
    bendA = (sx, gy)
    vstA = via_stack(sx, sy, 0, v_layer)        # creates via stack from layer 0 to V_layer where we start
    if stack_ok(vstA):
        p1A = segment((sx, sy), bendA, v_layer)  # vertical segment from start to bend 
        if p1A is not None:
            vbA = via_stack(sx, gy, v_layer, h_layer)      # create via stack from bend down to h_layer
            if stack_ok(vbA):
                p2A = segment(bendA, (gx, gy), h_layer)    # horizontal segment from bend to goal
                if p2A is not None:
                    vgoA = via_stack(gx, gy, h_layer, 0)   # via stack from goal (on h_layer) to m2
                    if stack_ok(vgoA):
                        # combines all subpaths (removing intersection duplicates)
                        pathA = (
                            [(sx, sy, 0)]
                            + vstA[1:]   # 0 -> v_layer
                            + p1A[1:]    # along v_layer
                            + vbA[1:]    # v_layer -> h_layer
                            + p2A[1:]    # along h_layer
                            + vgoA[1:]   # h_layer -> 0
                        )
                        if pathA[0][2] == 0 and pathA[-1][2] == 0:
                            return pathA

    # Option B: horizontal (odd) then vertical (even): bend at (gx, sy)
    bendB = (gx, sy)
    sthB = via_stack(sx, sy, 0, h_layer)
    if stack_ok(sthB):
        q1B = segment((sx, sy), bendB, h_layer)  # horizontal
        if q1B is not None:
            qbB = via_stack(gx, sy, h_layer, v_layer)
            if stack_ok(qbB):
                q2B = segment(bendB, (gx, gy), v_layer)  # vertical
                if q2B is not None:
                    qgoB = via_stack(gx, gy, v_layer, 0)
                    if stack_ok(qgoB):
                        pathB = (
                            [(sx, sy, 0)]
                            + sthB[1:]   # 0 -> h_layer
                            + q1B[1:]    # along h_layer
                            + qbB[1:]    # h_layer -> v_layer
                            + q2B[1:]    # along v_layer
                            + qgoB[1:]   # v_layer -> 0
                        )
                        if pathB[0][2] == 0 and pathB[-1][2] == 0:
                            return pathB

    return None

def try_l_route_with_layer_fallback(start, goal, routing_db, num_layers):
    """
    Attempts L-routing starting with the highest available metal layers and progressively decrements layers.
    Returns a path for the first L route that succeeeds, or None if all attempts fail.
    """
    # generate consecutive layer pairs starting from highest: (6,7), (5,6), (4,5), (3,4), (2,3), (1,2)
    # we skip (0,1) since layer 0 is M2 which is reserved for pins
    layer_pairs = []
    for high_layer in range(num_layers - 1, 1, -1):  # (7,6), (6,5), ..., (2,1)
        low_layer = high_layer - 1
        if 0 <= low_layer < num_layers:
            layer_pairs.append((low_layer, high_layer))
    
    # try L route on each layer pair in order (highest to lowest)
    for layer_pair in layer_pairs:
        result = try_l_shape(start, goal, routing_db, layers=layer_pair)
        if result is not None:
            return result
    
    return None


def run_routing(netlist, params, initial_order=None):
    """Perform a single routing pass using the existing loop logic.

    The implementation is essentially the same code that was previously in the
    "BEGIN MAIN" section but packaged into a callable function so that we can
    rerun with a different net ordering if desired.
    """
    start_time = time.perf_counter()

    netlist = deepcopy(netlist)

    # build order, prioritizing an explicit list if provided
    if initial_order is None:
        routing_order = create_routing_order(netlist)
    else:
        seen = set()
        routing_order = []
        for n in initial_order:
            if n in netlist["nets"] and n not in seen:
                routing_order.append(n)
                seen.add(n)
        for n in netlist["nets"]:
            if n not in seen:
                routing_order.append(n)
    total_nets = len(routing_order)

    work_q = deque(routing_order)   # routing order implemented as a deque so that we can efficiently pop from the left and append to the right as we do reroutes. 
    in_queue = set(routing_order)   # set to keep track of which nets are currently in the queue for O(1) membership checks, since checking membership in a deque is O(n)
    routed_set = set()
    routed_order = []
    reroute_count = {}

    routing_db = RoutingDB(grid_size=netlist['grid_size'], params=params)

    # mark pin locations in routing_db.pin_occ so that A* can recognize them and route through them, but not place wires on top of them.
    for net_name, net in netlist["nets"].items():
        for (x, y) in net["pins"]:
            idx = routing_db.coordinate_to_idx(x, y, 0)
            routing_db.pin_occ[idx] = 1

    pops = 0
    max_total_pops = 10 * len(routing_order)    # sets maximum amount of pops allowed to prevent infinite looping
    permanent_fail = set()                      # nets marked as permanently failed due to too many reroutes or time limit exceeded
    trouble_nets = set()                        # nets that have exceeded time limit for routing but have not yet hit max reroute attempts, so they are marked as "trouble" but not yet a permanent fail. This is useful to track separately since these nets are likely to be the main contributors to routing failure, and we may want to analyze them further.

    # pulse functionality to print out intermediate stats every PULSE_S seconds
    t0 = time.perf_counter()
    last_pulse = t0
    PULSE_S = 10.0

    while work_q and pops < max_total_pops:
        now = time.perf_counter()

        # print pulse with intermediate stats
        if now - last_pulse > PULSE_S:
            print(f"[{now-t0}s] routed={len(routed_set)}/{total_nets} "
                  f"queue={len(work_q)} pops={pops} current={work_q[0]}")
            last_pulse = now

        net_name = work_q.popleft()
        in_queue.remove(net_name)
        pops += 1

        # skip nets that have been marked as permanent fails so that we dont waste time trying to route them again.
        if net_name in permanent_fail:
            continue

        (x0, y0), (x1, y1) = netlist["nets"][net_name]["pins"]  # unpacking pins
        start = (x0, y0, 0)
        goal = (x1, y1, 0)

        # create a local copy of params for this net. **vars(params) turns the dataclass into a dictionary, which is then unpacked into the RouterParams parameters to create a new instance. This allows us to modify local_params without affecting the original params object that is shared across nets.
        local_params = RouterParams(**vars(params))    
        detailed_route = None

        # attempt to create L routes on higher metal layers before attempting detailed A*. This is an optimization that can save time on simple routes at the cost of potentially unnecessary vias
        quick = try_l_route_with_layer_fallback(start, goal, routing_db, local_params.num_layers)
        if quick is not None:
            detailed_route = quick

        net_start_time = time.perf_counter()
        for attempt in range(local_params.max_reroute_attempts):
            # tracking how much time we have spent on this net across all attempts, and if it exceeds the max_time_per_net parameter, we mark it as a trouble net (and permanently fail it) and break out of the loop to avoid wasting more time on it.
            if time.perf_counter() - net_start_time > local_params.max_time_per_net:
                print(f"Net {net_name} exceeded time limit ({local_params.max_time_per_net}s); \
                      marking as trouble and giving up on further attempts.")
                trouble_nets.add(net_name)
                permanent_fail.add(net_name)
                break

            if detailed_route is not None:      # if we found a route through L-routing, then skip A* attempts. 
                break

            # do global routing if the net is the correct type
            if netlist["nets"][net_name]["type"] in local_params.do_global_for_types:
                global_route = A_star_global(start, goal, routing_db, params=local_params, endpoints_are_tiles=False)
            else:
                global_route = None

            if global_route is None:
                band_limit = None
            else:
                # if global route is present, we use it to determine the band_limit for detailed A*. The band_limit determines how far outside the global route corridor the detailed A* is allowed to search. We set it to the current attempt number to allow the search to expand further outside the corridor on each subsequent attempt, up to a maximum defined by the length of local_params.corridor_band_cost.
                band_limit = min(attempt, len(local_params.corridor_band_cost) - 1)

            # if we have already attempted at least one detailed A* with the global route and if we allow relaxing the corridor on retries, then we set the global route to None to allow the detailed A* to search more freely without being guided by the global route
            if attempt > 0 and local_params.relax_corridor_on_retry:
                global_route = None

            detailed_route = A_star_detailed(start, goal, routing_db, global_route,
                                             params=local_params,
                                             band_limit=(band_limit if band_limit is not None else math.inf))

            if detailed_route is not None:
                if attempt > 0:
                    print(f"Successfully found path between {start} and {goal} after {attempt} attempts.")      # only print this message if we have found a path after previously failing
                break

            # select nets to be ripped up based on the neighborhood around the start and goal and recency of routing, and then rip them up and add them back to the queue if they have not exceeded the max_requeue_per_net limit
            rip_list = choose_ripup_nets(routing_db, routed_order, start, goal, k=local_params.ripup_k)
            for rip_net in rip_list:
                if rip_net not in routed_set:   # sanity check to make sure the net we want to rip up is actually routed
                    continue
                routing_db.rip_up(rip_net)      # rip up the net in the routing database
                routed_set.remove(rip_net)      # remove it from the set of routed nets
                routed_order.remove(rip_net)    # remove it from the order of routed nets. This is important for the recency ranking in choose_ripup_nets to work correctly on subsequent attempts.
                cnt = reroute_count.get(rip_net, 0) + 1     # increment the reroute count for this net since we are about to attempt to reroute it again
                reroute_count[rip_net] = cnt                # update the reroute count in the dictionary

                # if the net we just ripped up has not exceeded the max_requeue_per_net limit, and if it is not already in the queue to be routed again, and if it has not been marked as a permanent fail, then we add it back to the queue to be rerouted. Otherwise, if it has exceeded the max_requeue_per_net limit, we mark it as a permanent fail so that we dont waste time trying to route it again in the future.
                if cnt <= local_params.max_requeue_per_net: 
                    if rip_net not in in_queue and rip_net not in permanent_fail:
                        work_q.append(rip_net)
                        in_queue.add(rip_net)
                else:
                    print(f"{rip_net} has failed to route {local_params.max_requeue_per_net} times. "
                          "Marking as a permanent fail.")
                    permanent_fail.add(rip_net)

            # update the history penalties based on the current congestion. This will affect the cost function for the next A* attempt, making it more likely to find a different path that is less congested.
            routing_db.update_history_penalties() 
            local_params.cong_alpha *= local_params.bump_cong_alpha_on_retry    # bumping up the congestion alpha to make congestion penalties more severe on each retry, which should encourage the search to find less congested paths on subsequent attempts

        # if we exit the attempt loop without finding a detailed route, then we check if the net should be requeued for another attempt or marked as a permanent fail. We only requeue it if it has not exceeded the max_requeue_per_net limit and if it is not already in the queue or marked as a permanent fail. Otherwise, we mark it as a permanent fail to avoid wasting time on it in the future
        if detailed_route is None:
            if net_name in trouble_nets:
                continue
            cnt = reroute_count.get(net_name, 0) + 1
            reroute_count[net_name] = cnt
            if cnt <= local_params.max_requeue_per_net:
                if net_name not in in_queue:
                    work_q.append(net_name)
                    in_queue.add(net_name)
            else:
                permanent_fail.add(net_name)
            continue

        routing_db.commit_route(net_name, detailed_route)       # commit the successful route to the routing database 
        routed_set.add(net_name)                                # add the net to the set of routed nets for O(1) membership checks
        routed_order.append(net_name)                           # add the net to the order of routed nets, which is used for recency ranking in choose_ripup_nets       

    end_time = time.perf_counter()
    elapsed = end_time - start_time         # total time elapsed for this routing pass
    print(f"Routing summary: {len(routed_set)}/{total_nets} nets routed "
          f"({100.0*len(routed_set)/total_nets}%). Failed: {total_nets - len(routed_set)}. "
          f"Permanent fails: {len(permanent_fail)}. Pops: {pops}.")
    if trouble_nets:
        print(f"Trouble nets (timed out): {trouble_nets}")

    return routing_db, routed_set, routed_order, permanent_fail, trouble_nets, pops, elapsed

"""

========================================================================================
BEGIN MAIN
========================================================================================
"""

netlist = deepcopy(data)
params = RouterParams(
    num_layers=8,
    tile_size=10,

    cong_alpha=8.0,
    cong_p=2.0,

    do_global_for_types=("LONG",),
    global_cong_weight=0.002, 
    global_turn_penalty=0.0,

    corridor_band_cost=[0.0, 1.0, 3.0, 6.0],
    corridor_far_penalty=3.0,
    corridor_util_thresh=0.02,

    max_reroute_attempts=10,
    ripup_k=10,
    relax_corridor_on_retry=True,
    bump_cong_alpha_on_retry=1.5,
    max_requeue_per_net=20,
)

# attempt routing once with the default order, and then if we see trouble nets and we finished reasonably quickly, we try rerouting just the trouble nets with a different order to see if we can get more of them routed successfully
routing_db, routed_set, routed_order, permanent_fail, trouble_nets, pops, elapsed = run_routing(netlist, params)

# if we finished reasonably quickly and saw trouble nets, try rerouting them first
# we check if the total elapsed time is less than twice the maximum time we would expect to spend on the trouble nets if we routed them sequentially with no interference. If we have already spent more time than that, then it is unlikely that rerouting just the trouble nets will finish in a reasonable time frame, so we skip the second pass to avoid wasting more time.
if trouble_nets and elapsed < len(trouble_nets) * params.max_time_per_net * 2:  
    print("\nRe-running routing with trouble nets prioritized...")
    second_db, second_routed_set, second_order, second_fail, second_trouble, second_pops, second_elapsed = run_routing(netlist, params, initial_order=list(trouble_nets))
    if len(second_routed_set) > len(routed_set):
        routing_db = second_db
        routed_set = second_routed_set
        permanent_fail = second_fail
        print("Second pass achieved more successful routes, adopting its result.")
    else:
        print("Second pass did not improve number of routed nets; keeping original solution.")
    elapsed += second_elapsed  # accumulate total time

success_routes = len(routed_set)
total_nets = len(netlist["nets"])
failed_routes = total_nets - success_routes

print(f"Routing summary: {success_routes}/{total_nets} nets routed "
      f"({100.0*success_routes/total_nets}%). Failed: {failed_routes}. "
      f"Permanent fails: {len(permanent_fail)}. Pops: {pops}.")
if trouble_nets:
    print(f"Trouble nets (timed out) : {trouble_nets}")

completed_routes = add_m1_vias_to_all_routes(routing_db, netlist)
print(f"Total cost for this netlist is {total_routing_cost_from_routes(completed_routes)}")

print(f"Time elapsed: {elapsed} seconds.")
