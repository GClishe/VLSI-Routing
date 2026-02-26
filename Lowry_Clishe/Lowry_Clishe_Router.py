#
#
#
#

#User parameters
DATA_NAME = 'Reval_1500_40000'          #Name of netlist file. Make sure original folder names are used and that result folders exist
MASTER_SEED = 123456789                 #Set seed to make RND reproducable.
NUM_LAYERS = 9                          #Set the number of layers available
maxPatternSize = -1                     #Set the maximum length pattern routing should be used for. Set to 0 to disable pattern routing step, and -1 for auto
ADDITIONAL_PATTERN_LAYERS = True        #Allow pattern router to use layers M2-M9 when enabled, or only M2-M5 when disabled
SUBOPTIMAL_PATTERNS = True              #Allow patterns to extend past the first layer they are allowed to be on, as well as enabling detouring patterns like U and Z. If disabled, ADDITIONAL_PATTERN_LAYERS won't do anything
GENERATE_GRAPHS = True                  #Enables generation of 3D graphs to show final route
SHOW_GRAPH = False                      #Enables an interactive version of the graph to appear when finished

#Set import / export folder names based on data set
if DATA_NAME[:2] == 'Re':                #If the name of the data set starts with Re
    FOLDER_NAME = 'Reval_netlists-2'     #Assume it's Reval and set it to that folder
else:                                   #Otherwise assume Rtest. Use else here to accomadate the one called netlist_100_100
    FOLDER_NAME = 'Rtest_netlists'       #Set folder to Rtest

#Import libraries
import numpy as np
import pprint
import time

#Set RNG Seed
rng = np.random.default_rng(MASTER_SEED)

#Format data from the library file to be easier to deal with
data = getattr(__import__(f'{FOLDER_NAME}.{DATA_NAME}', fromlist = ['data']), 'data')     #Import net list data
GRID_SIZE = data['grid_size']        #Extract grid size
NET_COUNT = len(data['nets'])        #Extract number of nets
netList = np.zeros((NET_COUNT, 7), dtype = np.int32)             #Create an empty array with 7 entries per net
for i in range(NET_COUNT):                                       #Fill the empty list with data about each net
    netList[i] = [int(data['nets'][f'NET_{i}']['pins'][0][0]),  #[0]Pin0X
                  int(data['nets'][f'NET_{i}']['pins'][0][1]),  #[1]Pin0Y
                  int(data['nets'][f'NET_{i}']['pins'][1][0]),  #[2]Pin1X
                  int(data['nets'][f'NET_{i}']['pins'][1][1]),  #[3]Pin1Y
                  int(data['nets'][f'NET_{i}']['length']),      #[4]HPWL
                  0,                                            #[5]Locked (0=unlocked, 1=locked)
                  0]                                            #[6]Final cost (0=unrouted)
segList = [0] * NET_COUNT   #Create an empty list filled with a lists for each nets (normal python list for resizability) segList[net][segment][0:start, 1:end, 2:type(0=Via, 1=Hor, 2=Vert)][0:x, 1:y, 2:layer]
layoutGrid = np.full((GRID_SIZE, GRID_SIZE, NUM_LAYERS, 5), -1, dtype = np.int32) #Create 3D array for layout and give each cell a few variables ([0]what net is on this cell, [1]what segment of net(prioratize start of next for overlap), [2]calculated cost from here to destination, [3]actual cost start to here, [4]net currently attempting route on cell


#Functions
def netCostStartToCell (x: int, y: int, z:int, net: int, seg: int) -> int:  #Find the actual cost of a net up until a given cell
    #Need net and segment because this can be called during layoutGrid updates. Still need to make sure segList is updated first
    #Follow that segment back to the start (segments can only go in one direction), go to prior segment, repeat
    #Handle first segment separately since you only do part of it
    direction = segList[net][seg][2]
    if direction == 0:          #Via
        cost = 2 * abs(segList[net][seg][0][2] - z)
    elif direction == 1:        #Hori
        cost = abs(segList[net][seg][0][0] - x)
    else:                       #Vert
        cost = abs(segList[net][seg][0][1] - y)
    for i in range(seg - 1, -1, -1):    #Step backwards through the segments
        direction = segList[net][i][2]
        if direction == 0:      #Via
            cost = cost + 2 * abs(segList[net][i][1][2] - segList[net][i][0][2])
        elif direction == 1:    #Hori
            cost = cost + abs(segList[net][i][1][0] - segList[net][i][0][0])
        else:                   #Vert
            cost = cost + abs(segList[net][i][1][1] - segList[net][i][0][1])
    return cost

def netCostCellToEnd(x: int, y: int, z:int, net:int) -> int:    #Find the predicted remaining cost of the net from this cell
    cost = abs(netList[net][2] - x) + abs(netList[net][3] - y) + z*2  #3D Manhattan z*2 for vias is fine for now
    return cost

def addVia (net: int, x:int, y:int, startLayer:int, endLayer:int):     #Adds a via segment to the segment list and layout grid. Updates cost
    global segList
    seg = len(segList[net])-1
    segList[net].insert(seg, [[x, y, startLayer], [x, y, endLayer], 0])  #Put new segment before end
    global layoutGrid
    layoutGrid[x][y][startLayer] = [net, seg, netCostCellToEnd(x,y, startLayer, net), netCostStartToCell(x, y, startLayer, net, seg), net]  #Fill in starting layer
    layoutGrid[x][y][endLayer] = [net, seg, netCostCellToEnd(x, y, endLayer, net), netCostStartToCell(x, y, endLayer, net, seg), net]       #Fill in destination layer
    layoutGrid[netList[net][2]][netList[net][3]][0][1] = seg + 1    #Update lower end via's segment ID on the layout grid
    layoutGrid[netList[net][2]][netList[net][3]][1][1] = seg + 1    #Update the segment ID of the upper part of the end via. This can overwrite the end of this via but the end takes priority
    if (x, y, endLayer) == (netList[net][2], netList[net][3], 1):   #If this via and the end via overlap
        cost = netCostStartToCell(netList[net][2], netList[net][3], 0, net, seg + 1)
        layoutGrid[netList[net][2]][netList[net][3]][0][3] =  cost  #Update cost start to cell of lower via
        netList[net][6] = cost                                      #Use this cost to update netList

def addHori (net: int, xStart:int, xEnd:int, y:int, z:int):     #Adds a horizontal segment to the segment list and layout grid. Updates cost
    global segList
    seg = len(segList[net])-1
    segList[net].insert(seg, [[xStart, y, z], [xEnd, y, z], 1])  #Put new segment before end
    global layoutGrid
    for x in range(xStart, xEnd + 1):   #Fill in cells
        layoutGrid[x][y][z] = [net, seg, netCostCellToEnd(x, y, z, net), netCostStartToCell(x, y, z, net, seg), net]
    layoutGrid[netList[net][2]][netList[net][3]][0][1] = seg + 1    #Update lower end via's segment ID on the layout grid
    layoutGrid[netList[net][2]][netList[net][3]][1][1] = seg + 1    #Update the segment ID of the upper part of the end via. This can overwrite the end of this segment but the end takes priority
    if (xEnd, y, z) == (netList[net][2], netList[net][3], 1):       #If this segment and the end via overlap
        cost = netCostStartToCell(netList[net][2], netList[net][3], 0, net, seg + 1)
        layoutGrid[netList[net][2]][netList[net][3]][0][3] =  cost  #Update cost start to cell of lower via
        netList[net][6] = cost                                      #Use this cost to update netList

def addVert (net: int, x:int, yStart: int, yEnd:int, z:int):     #Adds a horizontal segment to the segment list and layout grid. Updates cost
    global segList
    seg = len(segList[net])-1
    segList[net].insert(seg, [[x, yStart, z], [x, yEnd, z], 2])  #Put new segment before end
    global layoutGrid
    for y in range(yStart, yEnd + 1):   #Fill in cells
        layoutGrid[x][y][z] = [net, seg, netCostCellToEnd(x, y, z, net), netCostStartToCell(x, y, z, net, seg), net]
    layoutGrid[netList[net][2]][netList[net][3]][0][1] = seg + 1    #Update lower end via's segment ID on the layout grid
    layoutGrid[netList[net][2]][netList[net][3]][1][1] = seg + 1    #Update the segment ID of the upper part of the end via. This can overwrite the end of this segment but the end takes priority
    if (x, yEnd, z) == (netList[net][2], netList[net][3], 1):       #If this segment and the end via overlap

        cost = netCostStartToCell(netList[net][2], netList[net][3], 0, net, seg + 1)
        layoutGrid[netList[net][2]][netList[net][3]][0][3] =  cost  #Update cost start to cell of lower via
        netList[net][6] = cost                                      #Use this cost to update netList

def segOpen (net:int, x: int, y: int, z: int, dest: int, dir: int) -> bool:
    stepIteration = 1
    if dir == 0:    #Via
        if (abs(z - dest) != 1):    #Check that via is only one long (No speedup from removing checks)
            print('Error: Attempted to place illegal via')
            return False
        if  dest < z:
            stepIteration = -1
        for step in range(z, dest + stepIteration, stepIteration):
            if (layoutGrid[x][y][int(step)][0] not in {-1, net}):   #Traditional != and != is slower
                return False
    elif dir == 1:      #Hori
        if z % 2 == 1:  #Check that horizontal segment is on an even layer (M1 = index 0)
            print('Error: Attempted to place horizontal segment on vertical layer')
            return False
        if dest < x:
            stepIteration = -1
        for step in range(x, dest + stepIteration, stepIteration):
            if layoutGrid[int(step)][y][z][0] not in {-1, net}:
                return False
    else:               #Vert
        if z % 2 == 0:  #Check that vertival segment is on an odd layer (M1 = index 0)
            print('Error: Attempted to place vertical segment on horizontal layer')
            return False
        if dest < y:
            stepIteration = -1
        for step in range(y, dest + stepIteration, stepIteration):
            if layoutGrid[x][int(step)][z][0] not in {-1, net}:
                return False
    return True
    #Maybe using linspace or the if not in are inefficient because this gets slow
#def costCellToLastSeg 
#Trace the currently tracing variables back to the last segment in the net
#Need to clear the currently tracing net variable for all cells at the end of each search



#Start routing
startTime = time.perf_counter()     #Start timer
for i in range(NET_COUNT):  #Fill the empty lists with first and last segment as well as a value to indicate type of segment
    segList[i] = [[[netList[i][0], netList[i][1], 0], [netList[i][0], netList[i][1], 1], 0], #First via from M1 to M2
                  [[netList[i][2], netList[i][3], 1], [netList[i][2], netList[i][3], 0], 0]] #Last via from M2 to M1 
for i in range(NET_COUNT):  #Manually add start and end vias for each net to layoutGrid
    layoutGrid[segList[i][0][0][0]][segList[i][0][0][1]][0] = [i, 0, netCostCellToEnd(segList[i][0][0][0], segList[i][0][0][1], 0, i), netCostStartToCell(segList[i][0][0][0], segList[i][0][0][1], 0, i, 0), i]
    layoutGrid[segList[i][0][0][0]][segList[i][0][0][1]][1] = [i, 0, netCostCellToEnd(segList[i][0][0][0], segList[i][0][0][1], 1, i), netCostStartToCell(segList[i][0][0][0], segList[i][0][0][1], 1, i, 0), i]
    layoutGrid[segList[i][1][0][0]][segList[i][1][0][1]][0] = [i, 1, netCostCellToEnd(segList[i][1][0][0], segList[i][1][0][1], 0, i), -1, i]
    layoutGrid[segList[i][1][0][0]][segList[i][1][0][1]][1] = [i, 1, netCostCellToEnd(segList[i][1][0][0], segList[i][1][0][1], 1, i), -1, i]

#Attempt line / L pattern match on layers M2-M5. Reserve M6-M9 for advanced routing. Lock any 1 length routes on M2 and M3, those are optimal
if maxPatternSize != 0:     #Skip pattern routing is max size is 0
    patternLength = 1       #Set initial max HPWL size to pattern route as 1
    maxSizeTrack = 0        #Initialize temporary value for tracking largest HPWL
    horiAttempt = 0
    horiFail = 0
    vertAttempt = 0
    vertFail = 0
    LAttempt = 0
    LFail = 0
    if maxPatternSize == -1:    #If max pattern size is adaptive
        for i in range(NET_COUNT):   #Iterate throught the nets
            if maxPatternSize < netList[i][4]:   #And find the largest HPWL
                maxPatternSize = netList[i][4]    #Update value
    while(1):
        nextSmallestLength = GRID_SIZE * 2   #Reset next smallest tracker
        for i in range(NET_COUNT):   #For all nets
            x0 = netList[i][0]  #I added these to make it faster, it didn't, but it made it a lot easier to read so I kept it
            y0 = netList[i][1]
            x1= netList[i][2]
            y1 = netList[i][3]
            hpwl = netList[i][4]
            #if netList[i][6] == 0:  #If the net is unrouted
            if hpwl == patternLength:          #If the HPWL of this net is small enough
                if x0 == x1:      #If the start and end are on the same x
                    vertAttempt = vertAttempt + 1
                    if segOpen(i, x0, y0, 1, y1, 2):   #Check vertical line from start to end on M2
                        print(f'Routed Net {i} with HPWL {patternLength} using a vertical line on M2')
                        addVert(i, x0, y0, y1, 1) 
                    elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0) and SUBOPTIMAL_PATTERNS: #If M2 vertical line fails, Check via from M2 to M3 and M3 to M4 at start and M4 to M3 and M3 to M2 at end
                        if segOpen(i, x0, y0, 3, y1, 2):  #Check vertical line on M4
                            print(f'Routed Net {i} with HPWL {patternLength} using a vertical line on M4')
                            addVia(i, x0, y0, 1, 2)
                            addVia(i, x0, y0, 2, 3)
                            addVert(i, x0, y0, y1, 3) 
                            addVia(i, x1, y1, 3, 2)
                            addVia(i, x1, y1, 2, 1) #If I come back to it, I should lock the optimal pattern routes
                    #maybe try X spaces to left/right U routes. Try below M5 before doing above M5 straights. Maybe even do M1 streight, M1 U, M2 streight, M2 U... and have U check with a detour up to the via cost of going to the next layer
                        elif ADDITIONAL_PATTERN_LAYERS:
                            if segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0): #If M4 vertical line fails, Check via from M4 to M5 and M5 to M6 at start and M6 to M5 and M5 to M4 at end
                                if segOpen(i, x0, y0, 5, y1, 2):  #Check vertical line on M6
                                    print(f'Routed Net {i} with HPWL {patternLength} using a vertical line on M6')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVert(i, x0, y0, y1, 5) 
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x0, y0, 6, 7, 0) and segOpen(i, x1, y1, 7, 6, 0) and segOpen(i, x1, y1, 6, 5, 0): #If M8 vertical line fails, Check via from M6 to M7 and M7 to M8 at start and M8 to M7 and M7 to M6 at end
                                    if segOpen(i, x0, y0, 7, y1, 2):  #Check vertical line on M6
                                        print(f'Routed Net {i} with HPWL {patternLength} using a vertical line on M8')
                                        addVia(i, x0, y0, 1, 2)
                                        addVia(i, x0, y0, 2, 3)
                                        addVia(i, x0, y0, 3, 4)
                                        addVia(i, x0, y0, 4, 5)
                                        addVia(i, x0, y0, 5, 6)
                                        addVia(i, x0, y0, 6, 7)
                                        addVert(i, x0, y0, y1, 7) 
                                        addVia(i, x1, y1, 7, 6)
                                        addVia(i, x1, y1, 6, 5)
                                        addVia(i, x1, y1, 5, 4)
                                        addVia(i, x1, y1, 4, 3)
                                        addVia(i, x1, y1, 3, 2)
                                        addVia(i, x1, y1, 2, 1)
                                    else: vertFail = vertFail + 1    
                                else: vertFail = vertFail + 1        
                            else: vertFail = vertFail + 1            
                        else: vertFail = vertFail + 1                
                    else:vertFail = vertFail + 1
                elif y0 == y1:    #If the start and end are on the same y
                    horiAttempt = horiAttempt + 1
                    if segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x1, y1, 2, 1, 0): #Check via at start from M2 to M3 and via at end from M3 to M2
                        if segOpen(i, x0, y0, 2, x1, 1):  #Try M3 horizontal line from start to end
                            print(f'Routed Net {i} with HPWL {patternLength} using a horizontal line on M3')
                            addVia(i, x0, y0, 1, 2)
                            addHori(i, x0, x1, y0, 2) 
                            addVia(i, x1, y1, 2, 1)
                        elif segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and SUBOPTIMAL_PATTERNS: #If M3 horizontal line fails, Check vias at start from M3 to M4 and M4 to M5, and end from M5 to M4 and M4 to M3
                            if segOpen(i, x0, y0, 4, x1, 1):  #Check horizontal line from start to end on M5
                                print(f'Routed Net {i} with HPWL {patternLength} using a horizontal line on M5')
                                addVia(i, x0, y0, 1, 2)
                                addVia(i, x0, y0, 2, 3)
                                addVia(i, x0, y0, 3, 4)
                                addHori(i, x0, x1, y0, 4) 
                                addVia(i, x1, y1, 4, 3)
                                addVia(i, x1, y1, 3, 2)
                                addVia(i, x1, y1, 2, 1)
                    #maybe try X spaces to left/right U routes
                            elif ADDITIONAL_PATTERN_LAYERS:
                                if segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x1, y1, 6, 5, 0): #If M5 horizontal line fails, Check vias at start from M5 to M6 and M6 to M7, and end from M7 to M6 and M6 to M5
                                    if segOpen(i, x0, y0, 6, x1, 1):  #Check horizontal line from start to end on M7
                                        print(f'Routed Net {i} with HPWL {patternLength} using a horizontal line on M7')
                                        addVia(i, x0, y0, 1, 2)
                                        addVia(i, x0, y0, 2, 3)
                                        addVia(i, x0, y0, 3, 4)
                                        addVia(i, x0, y0, 4, 5)
                                        addVia(i, x0, y0, 5, 6)
                                        addHori(i, x0, x1, y0, 6) 
                                        addVia(i, x1, y1, 6, 5)
                                        addVia(i, x1, y1, 5, 4)
                                        addVia(i, x1, y1, 4, 3)
                                        addVia(i, x1, y1, 3, 2)
                                        addVia(i, x1, y1, 2, 1)
                                    elif segOpen(i, x0, y0, 6, 7, 0) and segOpen(i, x1, y1, 7, 6, 0) and segOpen(i, x0, y0, 7, 8, 0) and segOpen(i, x1, y1, 8, 7, 0): #If M7 horizontal line fails, Check vias at start from M7 to M8 and M8 to M9, and end from M9 to M8 and M8 to M3
                                        if segOpen(i, x0, y0, 8, x1, 1):  #Check horizontal line from start to end on M9
                                            print(f'Routed Net {i} with HPWL {patternLength} using a horizontal line on M9')
                                            addVia(i, x0, y0, 1, 2)
                                            addVia(i, x0, y0, 2, 3)
                                            addVia(i, x0, y0, 3, 4)
                                            addVia(i, x0, y0, 4, 5)
                                            addVia(i, x0, y0, 5, 6)
                                            addVia(i, x0, y0, 6, 7)
                                            addVia(i, x0, y0, 7, 8)
                                            addHori(i, x0, x1, y0, 8) 
                                            addVia(i, x1, y1, 8, 7)
                                            addVia(i, x1, y1, 7, 6)
                                            addVia(i, x1, y1, 6, 5)
                                            addVia(i, x1, y1, 5, 4)
                                            addVia(i, x1, y1, 4, 3)
                                            addVia(i, x1, y1, 3, 2)
                                            addVia(i, x1, y1, 2, 1)
                                        else: horiFail = horiFail + 1
                                    else: horiFail = horiFail + 1  
                                else: horiFail = horiFail + 1           
                            else:horiFail= horiFail + 1                  
                        else:horiFail = horiFail + 1                    
                    else:horiFail = horiFail + 1
                   
                else:                                   #Start and End point don't share x or y axis
                    LAttempt = LAttempt + 1
                    if segOpen(i, x0, y0, 1, y1, 2) and segOpen(i, x0, y1, 1, 2, 0) and segOpen(i, x1, y1, 2, 1, 0): #Check the vertical path on M2 to intersection, and via from M2 to M3 at intersection, and via on end point from M3 to M2
                        if segOpen(i, x0, y1, 2, x1, 1): #Check horizontal path from intersection to end on M3
                            print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M2 and M3')
                            addVert(i, x0, y0, y1, 1)
                            addVia(i, x0, y1, 1, 2)
                            addHori(i, x0, x1, y1, 2)
                            addVia(i, x1, y1, 2, 1)
                        else: LFail = LFail + 1
                    elif segOpen(i, x0, y0, 2, x1, 1) and segOpen(i, x0, y0, 1, 2, 0):      #Check via at start from M2 to M3, and horizontal segment on M3 from start to intersection
                        if segOpen(i, x1, y0, 1, y1, 2) and segOpen(i, x1, y0, 2, 1, 0):    #Check via at intersection from M3 to M2, and vertical segment on M2
                            print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M3 and M2')
                            addVia(i, x0, y0, 1, 2)
                            addHori(i, x0, x1, y0, 2)
                            addVia(i, x1, y0, 2, 1)
                            addVert(i, x1, y0, y1, 1)
                        else: LFail = LFail + 1
                    elif SUBOPTIMAL_PATTERNS:
                        if segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, x1, 1) and segOpen(i, x1, y0, 2, 3, 0) and segOpen(i, x1, y0, 3, y1, 2) and segOpen(i, x1, y1, 3, 2) and segOpen(i, x1, y1, 2, 1): #Check via at start from M2 to M3, and horizontal segment on M3 from start to intersection, and via at intersection from M3 to M4, and M4 vertical segment, and via from M4 to M3, and via from M3 to M2
                            print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M3 and M4')
                            addVia(i, x0, y0, 1, 2)
                            addHori(i, x0, x1, y0, 2)
                            addVia(i, x1, y0, 2, 3)
                            addVert(i, x1, y0, y1, 3)
                            addVia(i, x1, y1, 3, 2)
                            addVia(i, x1, y1, 2, 1)
                        elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, x1, 1) and segOpen(i, x1, y0, 4, 3, 0) and segOpen(i, x1, y0, 3, 2, 0) and segOpen(i, x1, y0, 2, 1, 0) and segOpen(i, x1, y0, 1, y1, 2): #Check via at start from M2 to M3, and via at start from M3 to M4, and via at Start from M4 to M5, and horizontal segment on M5 from start to intersection, and via at intersection from M5 to M4, and M4 to M3, and M3 to M2, and vertical segment on M2 from intersection to end
                            print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M5 and M2')
                            addVia(i, x0, y0, 1, 2)
                            addVia(i, x0, y0, 2, 3)
                            addVia(i, x0, y0, 3, 4)
                            addHori(i, x0, x1, y0, 4)
                            addVia(i, x1, y0, 4, 3)
                            addVia(i, x1, y0, 3, 2)
                            addVia(i, x1, y0, 2, 1)
                            addVert(i, x1, y0, y1, 1)
                        else: LFail = LFail + 1
                         #       
                         #       print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M5 and M4')
                    else: LFail = LFail + 1
 
                        #Can optimize L's by tracking previously checked segments
                        #Yeah I'm not doing z routes... unless... nah nevermind... weelllllll
            if hpwl < nextSmallestLength and hpwl > patternLength:  #Every iteration scan each net to find the next smallest value
                    nextSmallestLength = hpwl
        if nextSmallestLength == patternLength: #If no next smallest was found  
            break   #End pattern routing
        elif nextSmallestLength > maxPatternSize:  #If the next smallest pattern size exceeds the mad pattern size
            break   #End pattern routing   
        patternLength = nextSmallestLength  #If the loop hasn't broken, commit nextSmallestLength to be the next attempted pattern length



#Maybe once it finishes it's first pass, it rips up anything that's blocking an unrouted start/end via on any layer.
#I don't want to prevent the first pass from covering vias since it might prevent good routes. Try it though, maybe the tests are built weird

#The addSegment functions are dumb. They will place the segment no matter what, and will complete the net if anything in the net touches the end via
#Need to check locations for validity before doing addSegment, and the last segment placed must be one that overlaps with the end via
#All the values in layout are usable for search as long as that first net value is left at -1 until final placement
#The cost Start to Cell functiton depends on a continuous path of sequential and properly oriented segments including the target cell
#Need to find a way to trace that temp net value back to the last placed segment, read its value, then add the distance between

#start with patern router for shorts. Try all the orientations for that length (line, L, and 7. Z might be too much) on M2 and M3 (lines on both, L/7 span both layers as is)
#Stat with length of 1 and lock any nets you can complete with this on M2. During the search, note the next lowest HPWL
#On the next size step, you'll need to verify that nothing is obstructing the path
#After shorts are done, do for longs. A* what's left. If I add rip up later, don't remove M1/M2 vias

#Optimize by only calculating predicted and actual cost on segment end points


#Compute, format, and export results
routedNets = 0                          #Find what % of nets were routed
for i in range(NET_COUNT):
    if netList[i][6] != 0:
        routedNets = routedNets + 1
finalCost = sum(entry[6] for entry in netList)
print(f'vertAttempt = {vertAttempt}, vertFail = {vertFail}, horiAttempt = {horiAttempt}, horiFail = {horiFail}, LAttempt = {LAttempt}, LFail = {LFail}')
print(f'Routed {100*routedNets/NET_COUNT}% of nets with a cost of {finalCost} in {(time.perf_counter() - startTime):.3f} seconds')  #Print % of nets that were routed and how long it took
print(f"Exporting reults to ./Lowry_Clishe_{FOLDER_NAME}/{DATA_NAME}.py ...")

output = {                              #Create output data structure, add grid size and final cost
    'meta': {
        'grid_size': GRID_SIZE,
        'layer_directions': {
        },
        'total_cost': finalCost
    },
    'nets': {
    }
}

for i in range(1, NUM_LAYERS):           #Populate layer information
    if i % 2 == 0:
        output['meta']['layer_directions'][f'M{i + 1}'] = 'H'
    else:
        output['meta']['layer_directions'][f'M{i + 1}'] = 'V'

for i in range(NET_COUNT):               #Populate net information
    output['nets'][f'NET_{i}'] = {
        'cost': netList[i][6], 
        'pins': [(netList[i][0], netList[i][1]), (netList[i][2], netList[i][3])],
        'segments':[]
        }
    for j in range(len(segList[i])):    #Populate segments
        output['nets'][f'NET_{i}']['segments'].append({'end': (segList[i][j][1][0], segList[i][j][1][1], f'M{1 + segList[i][j][1][2]}'), 'start': (segList[i][j][0][0], segList[i][j][0][1], f'M{1 + segList[i][j][0][2]}')})

output_content = "data = " + pprint.PrettyPrinter(indent=4).pformat(output)             # Format output data
with open(f'Lowry_Clishe_{FOLDER_NAME}/{DATA_NAME}.py', 'w') as f:                        # Open export location
    f.write(output_content)                                                             # Export data
if GENERATE_GRAPHS:
    #Plot
    print('Graphing results...')
    from mpl_toolkits import mplot3d
    import matplotlib.pyplot as plt

    fig = plt.figure()              #Initialize plot
    ax = plt.axes(projection='3d')  #Set it to 3D
    ax.set_zlim([1,9])              #Show M1-M9 on z axis
    ax.set_xlim([0, GRID_SIZE])      #Limit to show grid
    ax.set_ylim([0, GRID_SIZE])
    colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan'] #List of line colors
    for i in range(NET_COUNT):       
        for j in range(len(segList[i])):    #Plot each segment of each net. Use a set color for the whole net
            ax.plot(np.linspace(segList[i][j][0][0], segList[i][j][1][0]), np.linspace(segList[i][j][0][1], segList[i][j][1][1]), np.linspace(segList[i][j][0][2], segList[i][j][1][2]) + 1, color=colors[i%10], label=f'Net {i}, Seg {j}')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Metal Layer')
    print(f'Saving graphs to ./Lowry_Clishe_{FOLDER_NAME}/{DATA_NAME}_View.png ...')
    ax.view_init(elev=90, azim=-90)     #Show top view
    plt.savefig(f"Lowry_Clishe_{FOLDER_NAME}/{DATA_NAME}_Top.png", dpi=400)       #Save plot
    ax.view_init(elev=0, azim=-90)      #Show front view
    plt.savefig(f"Lowry_Clishe_{FOLDER_NAME}/{DATA_NAME}_Front.png", dpi=400)     #Save plot
    ax.view_init(elev=0, azim=0)        #Show side view
    plt.savefig(f"Lowry_Clishe_{FOLDER_NAME}/{DATA_NAME}_Side.png", dpi=400)      #Save plot
    ax.view_init(elev=45, azim=-45)     #Show isometric view
    plt.savefig(f"Lowry_Clishe_{FOLDER_NAME}/{DATA_NAME}_Isometric.png", dpi=400) #Save plot
    if SHOW_GRAPH:
        print('Displaying graph...')
        plt.show()                      #Show plot