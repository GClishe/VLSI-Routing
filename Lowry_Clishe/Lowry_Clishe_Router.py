#
#
#
#

#User parameters
DATA_NAME = 'Reval_1000_30000'          #Name of netlist file. Make sure original folder names are used and that result folders exist
NUM_LAYERS = 9                          #Set the number of layers available
MAX_PATTERN_SIZE = -1                   #Set the maximum HPWL that pattern routing should be used for. Set to 0 to disable pattern routing, and -1 to automatically scale it to the largest HPWL
ADDITIONAL_PATTERN_LAYERS = True        #Allow pattern router to use layers M2-M9 when enabled, or only M2-M5 when disabled
SUBOPTIMAL_PATTERNS = True              #Allow patterns to extend past the first layer they are allowed to be on, as well as enabling detouring patterns like U and Z (not implimented). If disabled, ADDITIONAL_PATTERN_LAYERS won't do anything
ROUTE_TIME_LIMIT = 0.015                #Seconds to allow for an A* attempt per HPWL
RIPUP_MAX_LAYER = 3                     #After first routing attempt, if there are still nets unrouted, it will clear any routes in the area above their start/end pins. How high should this go. Ex 3 = rip up M3
NUM_ITERATIONS = 5                      #Number of times to try riping up and rerouting. This routing will stop early if it completes
GENERATE_GRAPHS = True                  #Enables generation of 3D graphs to show final route
SHOW_GRAPH = True                       #Enables an interactive version of the graph to appear when finished

#Set import / export folder names based on data set
if DATA_NAME[:2] == 'Re':               #If the name of the data set starts with Re
    FOLDER_NAME = 'Reval_netlists-2'    #Assume it's Reval and set it to that folder
else:                                   #Otherwise assume Rtest. Use else here to accomadate the one called netlist_100_100
    FOLDER_NAME = 'Rtest_netlists'      #Set folder to Rtest

#Import libraries
import numpy as np
import pprint
import time

#Format data from the library file to be easier to deal with
data = getattr(__import__(f'{FOLDER_NAME}.{DATA_NAME}', fromlist = ['data']), 'data')     #Import net list data
GRID_SIZE = data['grid_size']        #Extract grid size
NET_COUNT = len(data['nets'])        #Extract number of nets
netList = np.zeros((NET_COUNT, 7), dtype = np.int32)            #Create an empty array with 7 entries per net
for i in range(NET_COUNT):                                      #Fill the empty list with data about each net
    netList[i] = [int(data['nets'][f'NET_{i}']['pins'][0][0]),  #[0]Pin0X
                  int(data['nets'][f'NET_{i}']['pins'][0][1]),  #[1]Pin0Y
                  int(data['nets'][f'NET_{i}']['pins'][1][0]),  #[2]Pin1X
                  int(data['nets'][f'NET_{i}']['pins'][1][1]),  #[3]Pin1Y
                  int(data['nets'][f'NET_{i}']['length']),      #[4]HPWL
                  0,                                            #[5]Locked (0=unlocked, 1=locked)
                  0]                                            #[6]Final cost (0=unrouted)
segList = [0] * NET_COUNT   #Create an empty list filled with one list for each net. These sublists will be populated with more sub lists containg segment data (normal python list instead of numpy for resizability) 
#segList[net][segment][0:start, 1:end, 2:type(0=Via, 1=Hor, 2=Vert)][0:x, 1:y, 2:layer]
layoutGrid = np.full((GRID_SIZE, GRID_SIZE, NUM_LAYERS, 5), -1, dtype = np.int32) #Create 3D array for layout and give each cell a few variables 
#[0]what net is on this cell, [1]what segment of net(prioratize start of next for overlap), [2]calculated cost from here to destination, [3]actual cost start to here, [4]net currently attempting route on cell
# Add this near your User Parameters
HISTORY_PENALTY = 2.0  # Weight of the historical penalty. Adjust between 1.0 and 5.0 for tuning.

# Add this right after you initialize layoutGrid
historyGrid = np.zeros((GRID_SIZE, GRID_SIZE, NUM_LAYERS), dtype = np.float32)


#Functions
def netCostStartToCell (x: int, y: int, z:int, net: int, seg: int) -> int:  #Find the actual cost of a net up until a given cell
    #Need net and segment because this can be called during layoutGrid updates. Still need to make sure segList is updated first
    #Follow that segment back to the start (segments can only go in one direction), go to prior segment, repeat
    direction = segList[net][seg][2]    #Handle first segment separately since you only do part of it
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
    return abs(netList[net][2] - x) + abs(netList[net][3] - y) + z*2  #3D Manhattan z*2 for vias is fine for now
#######################################################
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
    stepIteration = 1
    if  xStart > xEnd:
        stepIteration = -1
    for x in range(xStart, xEnd + stepIteration, stepIteration):   #Fill in cells
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
    stepIteration = 1
    if  yStart > yEnd:
        stepIteration = -1
    for y in range(yStart, yEnd + stepIteration, stepIteration):   #Fill in cells
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
            print(f'Error: Attempted to place illegal via {net}')
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

def tallyRouted() -> int:
    routedNets = 0                          #Find how many nets were routed
    for i in range(NET_COUNT):
        if netList[i][6] != 0:
            routedNets = routedNets + 1
    return routedNets

def patternRouter():    #Attempt pattern routing
    global layoutGrid, netList, segList
    maxPatternSize = MAX_PATTERN_SIZE
    patternLength = 1       #Set initial max HPWL size to pattern route as 1
    horiAttempt = horiFail = vertAttempt = vertFail = LAttempt = LFail = 0
    if maxPatternSize == -1:                    #If max pattern size is adaptive
        for i in range(NET_COUNT):              #Iterate throught the nets
            if maxPatternSize < netList[i][4]:  #And find the largest HPWL
                maxPatternSize = netList[i][4]  #Update value
    while(1):
        nextSmallestLength = GRID_SIZE * 2      #Reset next smallest tracker
        for i in range(NET_COUNT):              #For all nets
            if netList[i][6] == 0:
                hpwl = netList[i][4]
                if hpwl == patternLength:           #If the HPWL of this net is small enough
                    x0 = netList[i][0]                  #I added these to make it faster, it didn't, but it made it a lot easier to read so I kept it
                    y0 = netList[i][1]
                    x1= netList[i][2]
                    y1 = netList[i][3]
                    if x0 == x1:                    #If the start and end are on the same x
                        vertAttempt = vertAttempt + 1
                        if segOpen(i, x0, y0, 1, y1, 2):        #Check vertical line from start to end on M2
                            print(f'Routed Net {i} with HPWL {patternLength} using a vertical line on M2')
                            addVert(i, x0, y0, y1, 1) 
                        elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0) and SUBOPTIMAL_PATTERNS: #If M2 vertical line fails, Check via from M2 to M3 and M3 to M4 at start and M4 to M3 and M3 to M2 at end
                            if segOpen(i, x0, y0, 3, y1, 2):    #Check vertical line on M4
                                print(f'Routed Net {i} with HPWL {patternLength} using a vertical line on M4')
                                addVia(i, x0, y0, 1, 2)
                                addVia(i, x0, y0, 2, 3)
                                addVert(i, x0, y0, y1, 3) 
                                addVia(i, x1, y1, 3, 2)
                                addVia(i, x1, y1, 2, 1)         
    #If I come back to it, I should lock the optimal pattern routes
    #maybe try X spaces to left/right U routes. Try below M5 before doing above M5 straights. Maybe even do M1 streight, M1 U, M2 streight, M2 U... and have U check with a detour up to the via cost of going to the next layer
    #Or try staggared routes
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
                            elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, y1, 2) and segOpen(i, x0, y1, 3, 2, 0) and segOpen(i, x0, y1, 2, x1, 1) and segOpen(i, x1, y1, 2, 1, 0): #Check via at start from M2 to M3 and M3 to M4, and vertical segment on M4 from start to intersection, and via at intersection from M4 to M3, and horizontal segment on M3 from intersection to end, and via at end from M3 to M2
                                print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M4 and M3')
                                addVia(i, x0, y0, 1, 2)
                                addVia(i, x0, y0, 2, 3)
                                addVert(i, x0, y0, y1, 3)
                                addVia(i, x0, y1, 3, 2)
                                addHori(i, x0, x1, y1, 2)
                                addVia(i, x1, y1, 2, 1)
                            elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, x1, 1) and segOpen(i, x1, y0, 4, 3, 0) and segOpen(i, x1, y0, 3, y1, 2) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, x1, 3, 2, 0): #Check via at start from M2 to M3, and via at start from M3 to M4, and via at Start from M4 to M5, and horizontal segment on M5 from start to intersection, and via at intersection from M5 to M4, and vertical segment on M4 from intersection to end, and vias at end from M4 to M3, and M3 to M2, 
                                print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M5 and M4')
                                addVia(i, x0, y0, 1, 2)
                                addVia(i, x0, y0, 2, 3)
                                addVia(i, x0, y0, 3, 4)
                                addHori(i, x0, x1, y0, 4)
                                addVia(i, x1, y0, 4, 3)
                                addVert(i, x1, y0, y1, 3) 
                                addVia(i, x1, y1, 3, 2)
                                addVia(i, x1, y1, 2, 1)
                            elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, y1, 2) and segOpen(i, x0, y1, 3, 4, 0) and segOpen(i, x0, y1, 4, x1, 1) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0): #Check vias at start from M2 to M3, and M3 to M4, and vertical segment from start to intersection on M4, and via at intersection from M4 to M5, and horizontal segment on M5 from intersection to end, and vias at end from M5 to M4, and M4 to M3, and M3 to M2
                                print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M4 and M5')
                                addVia(i, x0, y0, 1, 2)
                                addVia(i, x0, y0, 2, 3)
                                addVert(i, x0, y0, y1, 3)
                                addVia(i, x0, y1, 3, 4)
                                addHori(i, x0, x1, y1, 4)
                                addVia(i, x1, y1, 4, 3)
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
                            elif segOpen(i, x0, y0, 1, y1, 2) and segOpen(i, x0, y1, 1, 2, 0) and segOpen(i, x0, y1, 2, 3, 0) and segOpen(i, x0, y1, 3, 4, 0) and segOpen(i, x0, y1, 4, x1, 1) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0): #Check vertical segment at start on M2, and vias at intersection from M2 to M3, and M3 to M4, and M4 to M5, and horizontal segment from intersection to end on M5, and vias at end from M5 to M4, and M4 to M3, and M3 to M2
                                print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M2 and M5')
                                addVert(i, x0, y0, y1, 1)
                                addVia(i, x0, y1, 1, 2)
                                addVia(i, x0, y1, 2, 3)
                                addVia(i, x0, y1, 3, 4)
                                addHori(i, x0, x1, y1, 4)
                                addVia(i, x1, y1, 4, 3)
                                addVia(i, x1, y1, 3, 2)
                                addVia(i, x1, y1, 2, 1)
                            elif ADDITIONAL_PATTERN_LAYERS:
                                if segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, y1, 2) and segOpen(i, x0, y1, 5, 4, 0) and segOpen(i, x0, y1, 4, x1, 1) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M6 and M5')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVert(i, x0, y0, y1, 5)
                                    addVia(i, x0, y1, 5, 4)
                                    addHori(i, x0, x1, y1, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, x1, 1) and segOpen(i, x1, y0, 4, 5, 0) and segOpen(i, x1, y0, 5, y1, 2) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M5 and M6')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addHori(i, x0, x1, y0, 4)
                                    addVia(i, x1, y0, 4, 5)
                                    addVert(i, x1, y0, y1, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, x1, 1) and segOpen(i, x1, y0, 2, 3, 0) and segOpen(i, x1, y0, 3, 4, 0) and segOpen(i, x1, y0, 4, 5, 0) and segOpen(i, x1, y0, 5, y1, 2) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M3 and M6')
                                    addVia(i, x0, y0, 1, 2)
                                    addHori(i, x0, x1, y0, 2)
                                    addVia(i, x1, y0, 2, 3)
                                    addVia(i, x1, y0, 3, 4)
                                    addVia(i, x1, y0, 4, 5)
                                    addVert(i, x1, y0, y1, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)   
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, y1, 2) and segOpen(i, x0, y1, 5, 4, 0) and segOpen(i, x0, y1, 4, 3, 0) and segOpen(i, x0, y1, 3, 2, 0) and segOpen(i, x0, y1, 2, x1, 1) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M6 and M3')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVert(i, x0, y0, y1, 5)
                                    addVia(i, x0, y1, 5, 4)
                                    addVia(i, x0, y1, 4, 3)
                                    addVia(i, x0, y1, 3, 2)
                                    addHori(i, x0, x1, y1, 2)
                                    addVia(i, x1, y1, 2, 1) 
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x0, y0, 6, x1, 1) and segOpen(i, x1, y0, 6, 5, 0) and segOpen(i, x1, y0, 5, y1, 2) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M7 and M6')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVia(i, x0, y0, 5, 6)
                                    addHori(i, x0, x1, y0, 6)
                                    addVia(i, x1, y0, 6, 5)
                                    addVert(i, x1, y0, y1, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, y1, 2) and segOpen(i, x0, y1, 5, 6, 0) and segOpen(i, x0, y1, 6, x1, 1) and segOpen(i, x1, y1, 6, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M6 and M7')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVert(i, x0, y0, y1, 5)
                                    addVia(i, x0, y1, 5, 6)
                                    addHori(i, x0, x1, y1, 6)
                                    addVia(i, x1, y1, 6, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x0, y0, 6, x1, 1) and segOpen(i, x1, y0, 6, 5, 0) and segOpen(i, x1, y0, 5, 4, 0) and segOpen(i, x1, y0, 4, 3, 0)and segOpen(i, x1, y0, 3, y1, 2) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M7 and M4')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVia(i, x0, y0, 5, 6)
                                    addHori(i, x0, x1, y0, 6)
                                    addVia(i, x1, y0, 6, 5)
                                    addVia(i, x1, y0, 5, 4)
                                    addVia(i, x1, y0, 4, 3)
                                    addVert(i, x1, y0, y1, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, y1, 2) and segOpen(i, x0, y1, 3, 4, 0) and segOpen(i, x0, y1, 4, 5, 0) and segOpen(i, x0, y1, 5, 6, 0) and segOpen(i, x0, y1, 6, x1, 1) and segOpen(i, x1, y1, 6, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M4 and M7')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVert(i, x0, y0, y1, 3)
                                    addVia(i, x0, y1, 3, 4)
                                    addVia(i, x0, y1, 4, 5)
                                    addVia(i, x0, y1, 5, 6)
                                    addHori(i, x0, x1, y1, 6)
                                    addVia(i, x1, y1, 6, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)  
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x0, y0, 6, x1, 1) and segOpen(i, x1, y0, 6, 5, 0) and segOpen(i, x1, y0, 5, 4, 0) and segOpen(i, x1, y0, 4, 3, 0) and segOpen(i, x1, y0, 3, 2, 0) and segOpen(i, x1, y0, 2, 1, 0) and segOpen(i, x1, y0, 1, y1, 2):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M7 and M2')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVia(i, x0, y0, 5, 6)
                                    addHori(i, x0, x1, y0, 6)
                                    addVia(i, x1, y0, 6, 5)
                                    addVia(i, x1, y0, 5, 4)
                                    addVia(i, x1, y0, 4, 3)
                                    addVia(i, x1, y0, 3, 2)
                                    addVia(i, x1, y0, 2, 1)
                                    addVert(i, x1, y0, y1, 1)
                                elif segOpen(i, x0, y0, 1, y1, 2) and segOpen(i, x0, y1, 1, 2, 0) and segOpen(i, x0, y1, 2, 3, 0) and segOpen(i, x0, y1, 3, 4, 0) and segOpen(i, x0, y1, 4, 5, 0) and segOpen(i, x0, y1, 5, 6, 0) and segOpen(i, x0, y1, 6, x1, 1) and segOpen(i, x1, y1, 6, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M2 and M7')
                                    addVert(i, x0, y0, y1, 1)
                                    addVia(i, x0, y1, 1, 2)
                                    addVia(i, x0, y1, 2, 3)
                                    addVia(i, x0, y1, 3, 4)
                                    addVia(i, x0, y1, 4, 5)
                                    addVia(i, x0, y1, 5, 6)
                                    addHori(i, x0, x1, y1, 6)
                                    addVia(i, x1, y1, 6, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)  
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x0, y0, 6, x1, 1) and segOpen(i, x1, y0, 6, 7, 0) and segOpen(i, x1, y0, 7, y1, 2) and segOpen(i, x1, y1, 7, 6, 0) and segOpen(i, x1, y1, 6, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M7 and M8')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVia(i, x0, y0, 5, 6)
                                    addHori(i, x0, x1, y0, 6)
                                    addVia(i, x1, y0, 6, 7)
                                    addVert(i, x1, y0, y1, 7)
                                    addVia(i, x1, y1, 7, 6)
                                    addVia(i, x1, y1, 6, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x0, y0, 6, 7, 0) and segOpen(i, x0, y0, 7, y1, 2) and segOpen(i, x0, y1, 7, 6, 0) and segOpen(i, x0, y1, 6, x1, 1) and segOpen(i, x1, y1, 6, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M8 and M7')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVia(i, x0, y0, 5, 6)
                                    addVia(i, x0, y0, 6, 7)
                                    addVert(i, x0, y0, y1, 7)
                                    addVia(i, x0, y1, 7, 6)
                                    addHori(i, x0, x1, y1, 6)
                                    addVia(i, x1, y1, 6, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, x1, 1) and segOpen(i, x1, y0, 4, 5, 0) and segOpen(i, x1, y0, 5, 6, 0) and segOpen(i, x1, y0, 6, 7, 0) and segOpen(i, x1, y0, 7, y1, 2) and segOpen(i, x1, y1, 7, 6, 0) and segOpen(i, x1, y1, 6, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M5 and M8')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addHori(i, x0, x1, y0, 4)
                                    addVia(i, x1, y0, 4, 5)
                                    addVia(i, x1, y0, 5, 6)
                                    addVia(i, x1, y0, 6, 7)
                                    addVert(i, x1, y0, y1, 7)
                                    addVia(i, x1, y1, 7, 6)
                                    addVia(i, x1, y1, 6, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x0, y0, 6, 7, 0) and segOpen(i, x0, y0, 7, y1, 2) and segOpen(i, x0, y1, 7, 6, 0) and segOpen(i, x0, y1, 6, 5, 0) and segOpen(i, x0, y1, 5, 4, 0) and segOpen(i, x0, y1, 4, x1, 1)  and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M8 and M5')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVia(i, x0, y0, 5, 6)
                                    addVia(i, x0, y0, 6, 7)
                                    addVert(i, x0, y0, y1, 7)
                                    addVia(i, x0, y1, 7, 6)
                                    addVia(i, x0, y1, 6, 5)
                                    addVia(i, x0, y1, 5, 4)
                                    addHori(i, x0, x1, y1, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, x1, 1) and segOpen(i, x1, y0, 2, 3, 0) and segOpen(i, x1, y0, 3, 4, 0) and segOpen(i, x1, y0, 4, 5, 0) and segOpen(i, x1, y0, 5, 6, 0) and segOpen(i, x1, y0, 6, 7, 0) and segOpen(i, x1, y0, 7, y1, 2) and segOpen(i, x1, y1, 7, 6, 0) and segOpen(i, x1, y1, 6, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M3 and M8')
                                    addVia(i, x0, y0, 1, 2)
                                    addHori(i, x0, x1, y0, 2)
                                    addVia(i, x1, y0, 2, 3)
                                    addVia(i, x1, y0, 3, 4)
                                    addVia(i, x1, y0, 4, 5)
                                    addVia(i, x1, y0, 5, 6)
                                    addVia(i, x1, y0, 6, 7)
                                    addVert(i, x1, y0, y1, 7)
                                    addVia(i, x1, y1, 7, 6)
                                    addVia(i, x1, y1, 6, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x0, y0, 6, 7, 0) and segOpen(i, x0, y0, 7, y1, 2) and segOpen(i, x0, y1, 7, 6, 0) and segOpen(i, x0, y1, 6, 5, 0) and segOpen(i, x0, y1, 5, 4, 0) and segOpen(i, x0, y1, 4, 3, 0) and segOpen(i, x0, y1, 3, 2, 0) and segOpen(i, x0, y1, 2, x1, 1) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M8 and M3')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVia(i, x0, y0, 5, 6)
                                    addVia(i, x0, y0, 6, 7)
                                    addVert(i, x0, y0, y1, 7)
                                    addVia(i, x0, y1, 7, 6)
                                    addVia(i, x0, y1, 6, 5)
                                    addVia(i, x0, y1, 5, 4)
                                    addVia(i, x0, y1, 4, 3)
                                    addVia(i, x0, y1, 3, 2)
                                    addHori(i, x0, x1, y1, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x0, y0, 6, 7, 0) and segOpen(i, x0, y0, 7, 8, 0) and segOpen(i, x0, y0, 8, x1, 1) and segOpen(i, x1, y0, 8, 7, 0) and segOpen(i, x1, y0, 7, y1, 2) and segOpen(i, x1, y1, 7, 6, 0) and segOpen(i, x1, y1, 6, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M9 and M8')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVia(i, x0, y0, 5, 6)
                                    addVia(i, x0, y0, 6, 7)
                                    addVia(i, x0, y0, 7, 8)
                                    addHori(i, x0, x1, y0, 8)
                                    addVia(i, x1, y0, 8, 7)
                                    addVert(i, x1, y0, y1, 7)
                                    addVia(i, x1, y1, 7, 6)
                                    addVia(i, x1, y1, 6, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x0, y0, 6, 7, 0) and segOpen(i, x0, y0, 7, y1, 2) and segOpen(i, x0, y1, 7, 8, 0) and segOpen(i, x0, y1, 8, x1, 1) and segOpen(i, x1, y1, 8, 7, 0) and segOpen(i, x1, y1, 7, 6, 0) and segOpen(i, x1, y1, 6, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M8 and M9')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVia(i, x0, y0, 5, 6)
                                    addVia(i, x0, y0, 6, 7)
                                    addVert(i, x0, y0, y1, 7)
                                    addVia(i, x0, y1, 7, 8)
                                    addHori(i, x0, x1, y1, 8)
                                    addVia(i, x1, y1, 8, 7)
                                    addVia(i, x1, y1, 7, 6)
                                    addVia(i, x1, y1, 6, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x0, y0, 6, 7, 0) and segOpen(i, x0, y0, 7, 8, 0) and segOpen(i, x0, y0, 8, x1, 1) and segOpen(i, x1, y0, 8, 7, 0) and segOpen(i, x1, y0, 7, 6, 0) and segOpen(i, x1, y0, 6, 5, 0) and segOpen(i, x1, y0, 5, y1, 2) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M9 and M6')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVia(i, x0, y0, 5, 6)
                                    addVia(i, x0, y0, 6, 7)
                                    addVia(i, x0, y0, 7, 8)
                                    addHori(i, x0, x1, y0, 8)
                                    addVia(i, x1, y0, 8, 7)
                                    addVia(i, x1, y0, 7, 6)
                                    addVia(i, x1, y0, 6, 5)
                                    addVert(i, x1, y0, y1, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, y1, 2) and segOpen(i, x0, y1, 5, 6, 0) and segOpen(i, x0, y1, 6, 7, 0) and segOpen(i, x0, y1, 7, 8, 0) and segOpen(i, x0, y1, 8, x1, 1) and segOpen(i, x1, y1, 8, 7, 0) and segOpen(i, x1, y1, 7, 6, 0) and segOpen(i, x1, y1, 6, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M6 and M9')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVert(i, x0, y0, y1, 5)
                                    addVia(i, x0, y1, 5, 6)
                                    addVia(i, x0, y1, 6, 7)
                                    addVia(i, x0, y1, 7, 8)
                                    addHori(i, x0, x1, y1, 8)
                                    addVia(i, x1, y1, 8, 7)
                                    addVia(i, x1, y1, 7, 6)
                                    addVia(i, x1, y1, 6, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x0, y0, 6, 7, 0) and segOpen(i, x0, y0, 7, 8, 0) and segOpen(i, x0, y0, 8, x1, 1) and segOpen(i, x1, y0, 8, 7, 0) and segOpen(i, x1, y0, 7, 6, 0) and segOpen(i, x1, y0, 6, 5, 0) and segOpen(i, x1, y0, 5, 4, 0) and segOpen(i, x1, y0, 4, 3, 0) and segOpen(i, x1, y0, 3, y1, 2) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M9 and M4')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVia(i, x0, y0, 5, 6)
                                    addVia(i, x0, y0, 6, 7)
                                    addVia(i, x0, y0, 7, 8)
                                    addHori(i, x0, x1, y0, 8)
                                    addVia(i, x1, y0, 8, 7)
                                    addVia(i, x1, y0, 7, 6)
                                    addVia(i, x1, y0, 6, 5)
                                    addVia(i, x1, y0, 5, 4)
                                    addVia(i, x1, y0, 4, 3)
                                    addVert(i, x1, y0, y1, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, y1, 2) and segOpen(i, x0, y1, 3, 4, 0) and segOpen(i, x0, y1, 4, 5, 0) and segOpen(i, x0, y1, 5, 6, 0) and segOpen(i, x0, y1, 6, 7, 0) and segOpen(i, x0, y1, 7, 8, 0) and segOpen(i, x0, y1, 8, x1, 1) and segOpen(i, x1, y1, 8, 7, 0) and segOpen(i, x1, y1, 7, 6, 0) and segOpen(i, x1, y1, 6, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M4 and M9')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVert(i, x0, y0, y1, 3)
                                    addVia(i, x0, y1, 3, 4)
                                    addVia(i, x0, y1, 4, 5)
                                    addVia(i, x0, y1, 5, 6)
                                    addVia(i, x0, y1, 6, 7)
                                    addVia(i, x0, y1, 7, 8)
                                    addHori(i, x0, x1, y1, 8)
                                    addVia(i, x1, y1, 8, 7)
                                    addVia(i, x1, y1, 7, 6)
                                    addVia(i, x1, y1, 6, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                elif segOpen(i, x0, y0, 1, 2, 0) and segOpen(i, x0, y0, 2, 3, 0) and segOpen(i, x0, y0, 3, 4, 0) and segOpen(i, x0, y0, 4, 5, 0) and segOpen(i, x0, y0, 5, 6, 0) and segOpen(i, x0, y0, 6, 7, 0) and segOpen(i, x0, y0, 7, 8, 0) and segOpen(i, x0, y0, 8, x1, 1) and segOpen(i, x1, y0, 8, 7, 0) and segOpen(i, x1, y0, 7, 6, 0) and segOpen(i, x1, y0, 6, 5, 0) and segOpen(i, x1, y0, 5, 4, 0) and segOpen(i, x1, y0, 4, 3, 0) and segOpen(i, x1, y0, 3, 2, 0) and segOpen(i, x1, y0, 2, 1, 0) and segOpen(i, x1, y0, 1, y1, 2):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M9 and M2')
                                    addVia(i, x0, y0, 1, 2)
                                    addVia(i, x0, y0, 2, 3)
                                    addVia(i, x0, y0, 3, 4)
                                    addVia(i, x0, y0, 4, 5)
                                    addVia(i, x0, y0, 5, 6)
                                    addVia(i, x0, y0, 6, 7)
                                    addVia(i, x0, y0, 7, 8)
                                    addHori(i, x0, x1, y0, 8)
                                    addVia(i, x1, y0, 8, 7)
                                    addVia(i, x1, y0, 7, 6)
                                    addVia(i, x1, y0, 6, 5)
                                    addVia(i, x1, y0, 5, 4)
                                    addVia(i, x1, y0, 4, 3)
                                    addVia(i, x1, y0, 3, 2)
                                    addVia(i, x1, y0, 2, 1)
                                    addVert(i, x1, y0, y1, 1)
                                elif segOpen(i, x0, y0, 1, y1, 2) and segOpen(i, x0, y1, 1, 2, 0) and segOpen(i, x0, y1, 2, 3, 0) and segOpen(i, x0, y1, 3, 4, 0) and segOpen(i, x0, y1, 4, 5, 0) and segOpen(i, x0, y1, 5, 6, 0) and segOpen(i, x0, y1, 6, 7, 0) and segOpen(i, x0, y1, 7, 8, 0) and segOpen(i, x0, y1, 8, x1, 1) and segOpen(i, x1, y1, 8, 7, 0) and segOpen(i, x1, y1, 7, 6, 0) and segOpen(i, x1, y1, 6, 5, 0) and segOpen(i, x1, y1, 5, 4, 0) and segOpen(i, x1, y1, 4, 3, 0) and segOpen(i, x1, y1, 3, 2, 0) and segOpen(i, x1, y1, 2, 1, 0):
                                    print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M2 and M9')
                                    addVert(i, x0, y0, y1, 1)
                                    addVia(i, x0, y1, 1, 2)
                                    addVia(i, x0, y1, 2, 3)
                                    addVia(i, x0, y1, 3, 4)
                                    addVia(i, x0, y1, 4, 5)
                                    addVia(i, x0, y1, 5, 6)
                                    addVia(i, x0, y1, 6, 7)
                                    addVia(i, x0, y1, 7, 8)
                                    addHori(i, x0, x1, y1, 8)
                                    addVia(i, x1, y1, 8, 7)
                                    addVia(i, x1, y1, 7, 6)
                                    addVia(i, x1, y1, 6, 5)
                                    addVia(i, x1, y1, 5, 4)
                                    addVia(i, x1, y1, 4, 3)
                                    addVia(i, x1, y1, 3, 2)
                                    addVia(i, x1, y1, 2, 1)
                                else: LFail = LFail + 1
                            else: LFail = LFail + 1
    #Can optimize L's by tracking previously checked segments
    #Yeah I'm not doing z routes... unless... nah nevermind... weelllllll  IDK, I think at that point I should do a real router
                        else: LFail = LFail + 1
                if hpwl < nextSmallestLength and hpwl > patternLength:  #Every iteration scan each net to find the next smallest value
                        nextSmallestLength = hpwl
        if nextSmallestLength == patternLength: #If no next smallest was found  
            break   #End pattern routing
        elif nextSmallestLength > maxPatternSize:  #If the next smallest pattern size exceeds the mad pattern size
            break   #End pattern routing   
        patternLength = nextSmallestLength  #If the loop hasn't broken, commit nextSmallestLength to be the next attempted pattern length
    return horiAttempt, horiFail, vertAttempt, vertFail, LAttempt, LFail
"""
def ripperUpper() -> int:
    global layoutGrid, segList, netList
    ripCount = 0
    for i in range(NET_COUNT):
        if netList[i][6] == 0:  #If a net failed to route
            for j in range(2, RIPUP_MAX_LAYER):  #Scan all the layers above the start/end vias
                if layoutGrid[netList[i][0]][netList[i][1]][j][0] != -1:    #If there's something there
                    conflictNet = layoutGrid[netList[i][0]][netList[i][1]][j][0]
                    print(f'Removing Net {conflictNet}')
                    posx = netList[conflictNet][0]    #step through the whole net and remove each cell from layoutGrid
                    posy = netList[conflictNet][1]
                    posz = 0
                    lastMove = 0
                    while (posx != netList[conflictNet][2] or posy != netList[conflictNet][3] or posz != 0):
                        layoutGrid[posx][posy][posz][0] = -1
                        if segList[conflictNet][layoutGrid[posx][posy][posz][1]][2] == 0:
                            if NUM_LAYERS > posz + 1 and layoutGrid[posx][posy][posz + 1][0] == conflictNet and lastMove != 6:
                                posz = posz + 1
                                lastMove = 5
                            elif 0 <= posz - 1 and layoutGrid[posx][posy][posz - 1][0] == conflictNet and lastMove != 5:
                                posz = posz - 1
                                lastMove = 6
                            else:
                                print(f'Error: Unable to trace Net {i}')
                                break
                        elif segList[conflictNet][layoutGrid[posx][posy][posz][1]][2] == 1:
                            if GRID_SIZE > posx + 1 and layoutGrid[posx + 1][posy][posz][0] == conflictNet and lastMove != 2:
                                posx = posx + 1
                                lastMove = 1
                            elif 0 <= posx - 1 and layoutGrid[posx - 1][posy][posz][0] == conflictNet and lastMove != 1:
                                posx = posx - 1
                                lastMove = 2
                            else:
                                print(f'Error: Unable to trace Net {i}')
                                break
                        elif segList[conflictNet][layoutGrid[posx][posy][posz][1]][2] == 2:    
                            if GRID_SIZE > posy + 1 and layoutGrid[posx][posy + 1][posz][0] == conflictNet and lastMove != 4:
                                posy = posy + 1
                                lastMove = 3
                            elif 0 <= posy - 1 and layoutGrid[posx][posy - 1][posz][0] == conflictNet and lastMove != 3:
                                posy = posy - 1
                                lastMove = 4
                            else:
                                print(f'Error: Unable to trace Net {i}')
                                break
                        else:
                            print(f'Error: Unable to trace Net {i}')
                            break
                    ripCount = ripCount + 1
                    netList[conflictNet][6] = 0 #Mark net as unrouted
                    segList[conflictNet].clear()  #clear seg list, re add start/end vias to segList and layoutGrid
                    segList[conflictNet] = [[[netList[conflictNet][0], netList[conflictNet][1], 0], [netList[conflictNet][0], netList[conflictNet][1], 1], 0], #First via from M1 to M2
                                            [[netList[conflictNet][2], netList[conflictNet][3], 1], [netList[conflictNet][2], netList[conflictNet][3], 0], 0]] #Last via from M2 to M1 
                    layoutGrid[segList[conflictNet][0][0][0]][segList[conflictNet][0][0][1]][0] = [conflictNet, 0, netCostCellToEnd(segList[conflictNet][0][0][0], segList[conflictNet][0][0][1], 0, i), netCostStartToCell(segList[conflictNet][0][0][0], segList[conflictNet][0][0][1], 0, i, 0), conflictNet]
                    layoutGrid[segList[conflictNet][0][0][0]][segList[conflictNet][0][0][1]][1] = [conflictNet, 0, netCostCellToEnd(segList[conflictNet][0][0][0], segList[conflictNet][0][0][1], 1, i), netCostStartToCell(segList[conflictNet][0][0][0], segList[conflictNet][0][0][1], 1, i, 0), conflictNet]
                    layoutGrid[segList[conflictNet][1][0][0]][segList[conflictNet][1][0][1]][0] = [conflictNet, 1, netCostCellToEnd(segList[conflictNet][1][0][0], segList[conflictNet][1][0][1], 0, i), -1, conflictNet]
                    layoutGrid[segList[conflictNet][1][0][0]][segList[conflictNet][1][0][1]][1] = [conflictNet, 1, netCostCellToEnd(segList[conflictNet][1][0][0], segList[conflictNet][1][0][1], 1, i), -1, conflictNet]
    return ripCount
"""

def ripperUpper() -> int:
    global layoutGrid, segList, netList
    nets_to_rip = set() # Use a set to prevent ripping up the same net multiple times
    
    # 1. Identify which nets are blocking the unrouted nets
    for i in range(NET_COUNT):
        if netList[i][6] == 0:  # If a net failed to route
            startX, startY = netList[i][0], netList[i][1]
            endX, endY = netList[i][2], netList[i][3]
            
            # A. Clear anything directly above the start/end pins (Expanded to all layers)
            for j in range(1, NUM_LAYERS): 
                conflict_start = layoutGrid[startX][startY][j][0]
                conflict_end = layoutGrid[endX][endY][j][0]
                if conflict_start not in (-1, i) and netList[conflict_start][5] == 0: # Check if locked
                    nets_to_rip.add(conflict_start)
                if conflict_end not in (-1, i) and netList[conflict_end][5] == 0:
                    nets_to_rip.add(conflict_end)
            
            # B. Clear a direct L-shaped path between the pins to guarantee a route lane
            # Trace Vertical on M2 (z=1)
            stepY = 1 if endY >= startY else -1
            for y in range(startY, endY + stepY, stepY):
                conflictNet = layoutGrid[startX][y][1][0]
                if conflictNet not in (-1, i) and netList[conflictNet][5] == 0:
                    nets_to_rip.add(conflictNet)
                    
            # Trace Horizontal on M3 (z=2)
            stepX = 1 if endX >= startX else -1
            for x in range(startX, endX + stepX, stepX):
                conflictNet = layoutGrid[x][endY][2][0]
                if conflictNet not in (-1, i) and netList[conflictNet][5] == 0:
                    nets_to_rip.add(conflictNet)
                    
    ripCount = 0
    
    # 2. Rip up the identified blocking nets
    for conflictNet in nets_to_rip:
        print(f'Removing Net {conflictNet} to clear congestion')
        
        # Trace and remove from layoutGrid (Kept your original tracing logic to ensure grid integrity)
        posx = netList[conflictNet][0]    
        posy = netList[conflictNet][1]
        posz = 0
        lastMove = 0
        
        while (posx != netList[conflictNet][2] or posy != netList[conflictNet][3] or posz != 0):
            layoutGrid[posx][posy][posz][0] = -1
            seg_idx = layoutGrid[posx][posy][posz][1]
            
            # --- NEW: Increment historical cost for this cell ---
            historyGrid[posx][posy][posz] += 1.0

            # Safety check: if seg_idx is -1, the grid is already cleared or corrupted, break to avoid crash
            if seg_idx == -1: break 
            
            if segList[conflictNet][seg_idx][2] == 0:
                if NUM_LAYERS > posz + 1 and layoutGrid[posx][posy][posz + 1][0] == conflictNet and lastMove != 6:
                    posz = posz + 1
                    lastMove = 5
                elif 0 <= posz - 1 and layoutGrid[posx][posy][posz - 1][0] == conflictNet and lastMove != 5:
                    posz = posz - 1
                    lastMove = 6
                else:
                    break
            elif segList[conflictNet][seg_idx][2] == 1:
                if GRID_SIZE > posx + 1 and layoutGrid[posx + 1][posy][posz][0] == conflictNet and lastMove != 2:
                    posx = posx + 1
                    lastMove = 1
                elif 0 <= posx - 1 and layoutGrid[posx - 1][posy][posz][0] == conflictNet and lastMove != 1:
                    posx = posx - 1
                    lastMove = 2
                else:
                    break
            elif segList[conflictNet][seg_idx][2] == 2:    
                if GRID_SIZE > posy + 1 and layoutGrid[posx][posy + 1][posz][0] == conflictNet and lastMove != 4:
                    posy = posy + 1
                    lastMove = 3
                elif 0 <= posy - 1 and layoutGrid[posx][posy - 1][posz][0] == conflictNet and lastMove != 3:
                    posy = posy - 1
                    lastMove = 4
                else:
                    break
            else:
                break
                
        ripCount = ripCount + 1
        netList[conflictNet][6] = 0 # Mark net as unrouted
        
        # Reset segList and layoutGrid for the ripped up net
        segList[conflictNet].clear()  
        segList[conflictNet] = [[[netList[conflictNet][0], netList[conflictNet][1], 0], [netList[conflictNet][0], netList[conflictNet][1], 1], 0], 
                                [[netList[conflictNet][2], netList[conflictNet][3], 1], [netList[conflictNet][2], netList[conflictNet][3], 0], 0]] 
        
        layoutGrid[segList[conflictNet][0][0][0]][segList[conflictNet][0][0][1]][0] = [conflictNet, 0, netCostCellToEnd(segList[conflictNet][0][0][0], segList[conflictNet][0][0][1], 0, conflictNet), netCostStartToCell(segList[conflictNet][0][0][0], segList[conflictNet][0][0][1], 0, conflictNet, 0), conflictNet]
        layoutGrid[segList[conflictNet][0][0][0]][segList[conflictNet][0][0][1]][1] = [conflictNet, 0, netCostCellToEnd(segList[conflictNet][0][0][0], segList[conflictNet][0][0][1], 1, conflictNet), netCostStartToCell(segList[conflictNet][0][0][0], segList[conflictNet][0][0][1], 1, conflictNet, 0), conflictNet]
        layoutGrid[segList[conflictNet][1][0][0]][segList[conflictNet][1][0][1]][0] = [conflictNet, 1, netCostCellToEnd(segList[conflictNet][1][0][0], segList[conflictNet][1][0][1], 0, conflictNet), -1, conflictNet]
        layoutGrid[segList[conflictNet][1][0][0]][segList[conflictNet][1][0][1]][1] = [conflictNet, 1, netCostCellToEnd(segList[conflictNet][1][0][0], segList[conflictNet][1][0][1], 1, conflictNet), -1, conflictNet]

    return ripCount
    

def aStarRouter():  #This function was AI Generated with Gemini set to 3.1 Pro with the prompt: Here is my code for a VLSI router. Can you make a function I can add after my initial pattern route attempt to try A* routing? 
    import heapq
    global layoutGrid, netList, segList
    
    routed_count = 0
    attempt_count = 0
    
    for net in range(NET_COUNT):
        # Only attempt to route nets that haven't been completed yet
        if netList[net][6] == 0:
            routeTime = time.perf_counter()
            attempt_count += 1
            startX, startY = netList[net][0], netList[net][1]
            endX, endY = netList[net][2], netList[net][3]
            
            # Start and End positions at the top of the M1->M2 vias (z = 1)
            start_pos = (startX, startY, 1)
            end_pos = (endX, endY, 1)
            came_from = {}
            g_score = {start_pos: 0}
            path_found = False
            open_set = []
            heapq.heappush(open_set, (0, 0, start_pos)) # (f_score, g_score, (x, y, z))
            
            # Heuristic: 3D Manhattan Distance (Prioritizing planar moves over vias)
            def heuristic(pos):
                return abs(pos[0] - endX) + abs(pos[1] - endY) + (abs(pos[2] - 1) * 2)
            
            while open_set:
                _, current_g, current = heapq.heappop(open_set)
                
                if current == end_pos:
                    path_found = True
                    break
                elif time.perf_counter() > routeTime + netList[net][4]* ROUTE_TIME_LIMIT:
                    print(f'Net {net} took too long to route')
                    break

                x, y, z = current
                
                neighbors = []
                
                # Planar moves based on alternating layer direction
                if z % 2 == 1: 
                    # Odd index layers (M2, M4...) are Vertical
                    if y + 1 < GRID_SIZE: neighbors.append(((x, y + 1, z), 1))
                    if y - 1 >= 0: neighbors.append(((x, y - 1, z), 1))
                else:          
                    # Even index layers (M3, M5...) are Horizontal
                    if x + 1 < GRID_SIZE: neighbors.append(((x + 1, y, z), 1))
                    if x - 1 >= 0: neighbors.append(((x - 1, y, z), 1))
                
                # Via moves (Up/Down)
                if z < NUM_LAYERS - 1:
                    neighbors.append(((x, y, z + 1), 2)) # Up
                if z > 1: # Restrict going down to z=0 (M1) to keep routing on M2+
                    neighbors.append(((x, y, z - 1), 2)) # Down
                    
                for neighbor_pos, move_cost in neighbors:
                    nx, ny, nz = neighbor_pos
                    
                    # Validate cell: It must be empty (-1) or already belong to this net
                    grid_val = layoutGrid[nx][ny][nz][0]
                    if grid_val != -1 and grid_val != net:
                        continue

                    # --- NEW: Fetch historical penalty for this cell ---
                    history_cost = historyGrid[nx][ny][nz] * HISTORY_PENALTY
                    
                    # --- UPDATED: Add history_cost to the tentative_g ---
                    tentative_g = current_g + move_cost + history_cost
                    
                    if neighbor_pos not in g_score or tentative_g < g_score[neighbor_pos]:
                        came_from[neighbor_pos] = current
                        g_score[neighbor_pos] = tentative_g
                        f_score = tentative_g + heuristic(neighbor_pos)
                        heapq.heappush(open_set, (f_score, tentative_g, neighbor_pos))
            
            # If a path was found, reconstruct and commit it
            if path_found:
                # Backtrack to build the coordinate path
                path = []
                curr = end_pos
                while curr in came_from:
                    path.append(curr)
                    curr = came_from[curr]
                path.append(start_pos)
                path.reverse()
                
                # Convert the individual cell steps into contiguous segments
                if len(path) > 1:
                    seg_start = 0
                    active_dir = -1 # 0: via, 1: horizontal, 2: vertical
                    
                    for i in range(1, len(path)):
                        dx = path[i][0] - path[i-1][0]
                        dy = path[i][1] - path[i-1][1]
                        dz = path[i][2] - path[i-1][2]
                        
                        if dz != 0: current_dir = 0
                        elif dx != 0: current_dir = 1
                        else: current_dir = 2
                            
                        if active_dir == -1:
                            active_dir = current_dir
                            
                        # When the direction changes, commit the previous segment to the layout
                        if current_dir != active_dir:
                            p_start = path[seg_start]
                            p_end = path[i-1]
                            
                            if active_dir == 0:
                                # Break multi-layer via jumps into individual 1-layer steps
                                for z_step in range(abs(p_end[2] - p_start[2])):
                                    z1 = p_start[2] + z_step * (1 if p_end[2] > p_start[2] else -1)
                                    z2 = z1 + (1 if p_end[2] > p_start[2] else -1)
                                    addVia(net, p_start[0], p_start[1], z1, z2)
                            elif active_dir == 1:
                                addHori(net, p_start[0], p_end[0], p_start[1], p_start[2])
                            elif active_dir == 2:
                                addVert(net, p_start[0], p_start[1], p_end[1], p_start[2])
                                
                            seg_start = i - 1
                            active_dir = current_dir
                            
                    # Commit the final segment of the net
                    p_start = path[seg_start]
                    p_end = path[-1]
                    if active_dir == 0:
                        for z_step in range(abs(p_end[2] - p_start[2])):
                            z1 = p_start[2] + z_step * (1 if p_end[2] > p_start[2] else -1)
                            z2 = z1 + (1 if p_end[2] > p_start[2] else -1)
                            addVia(net, p_start[0], p_start[1], z1, z2)
                    elif active_dir == 1:
                        addHori(net, p_start[0], p_end[0], p_start[1], p_start[2])
                    elif active_dir == 2:
                        addVert(net, p_start[0], p_start[1], p_end[1], p_start[2])
                        
                print(f"Routed Net {net} using A*")
                routed_count += 1
            else:
                print(f"A* failed to find a path for Net {net}")
                
    if attempt_count > 0:
        return attempt_count, routed_count
    else:
        return 0, 0
      

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

if MAX_PATTERN_SIZE != 0:
    horiAttempt, horiFail, vertAttempt, vertFail, LAttempt, LFail = patternRouter() #Pattern route what you can
    patternTime = time.perf_counter() - startTime
else: 
    horiAttempt = horiFail = vertAttempt = vertFail = LAttempt = LFail = NET_COUNT
    patternTime = 0

if tallyRouted() < NET_COUNT:
    aStarAttempts = np.zeros((NUM_ITERATIONS + 1), dtype = np.int32)
    aStartRouted = np.zeros((NUM_ITERATIONS + 1), dtype = np.int32)
    aStarTime = np.zeros((NUM_ITERATIONS + 1), dtype = np.float32)
    aStarAttempts[0], aStartRouted[0] = aStarRouter()   #A* the rest
    aStarTime[0] = time.perf_counter() - patternTime - startTime
    if NUM_ITERATIONS > 0 and tallyRouted() < NET_COUNT:
        ripCount = np.zeros((NUM_ITERATIONS), dtype = np.int32)
        ripTime = np.zeros((NUM_ITERATIONS + 1), dtype = np.float32)
        for i in range(NUM_ITERATIONS):
            if tallyRouted() < NET_COUNT:
                ripCount[i] = ripperUpper()
                ripTime[i + 1] = time.perf_counter() - patternTime - startTime
                for j in range(i + 1):
                    ripTime[i + 1] = ripTime[i + 1] - aStarTime[j] - ripTime[j]
                aStarAttempts[i + 1], aStartRouted[i + 1] = aStarRouter()
                aStarTime[i + 1] = time.perf_counter() - patternTime - startTime
                for j in range(i + 1):
                    aStarTime[i + 1] = aStarTime[i + 1] - aStarTime[j] - ripTime[j + 1]

#Maybe once it finishes it's first pass, it rips up anything that's blocking an unrouted start/end via on any layer.
#I don't want to prevent the first pass from covering vias since it might prevent good routes. Try it though, maybe the tests are built weird

#The addSegment functions are dumb. They will place the segment no matter what, and will complete the net if anything in the net touches the end via
#Need to check locations for validity before doing addSegment, and the last segment placed must be one that overlaps with the end via
#All the values in layout are usable for search as long as that first net value is left at -1 until final placement
#The cost Start to Cell functiton depends on a continuous path of sequential and properly oriented segments including the target cell

#Compute results
routedNets = tallyRouted()
finalCost = sum(entry[6] for entry in netList)  #Find the final cost
print('')
if MAX_PATTERN_SIZE != 0:
    print(f'Pattern router completed {vertAttempt - vertFail}/{vertAttempt} vertical line routes, {horiAttempt- horiFail}/{horiAttempt} horizontal line routes, and {LAttempt-LFail}/{LAttempt} L routes in {(patternTime):.3f} seconds') #Print summary of unrouted nets
if (vertFail + horiFail + LFail) != 0:
    print(f'A* pass 1 completed {aStartRouted[0]}/{aStarAttempts[0]} of the remaining routes in {aStarTime[0]:.3f} seconds')
    if aStarAttempts[0] - aStartRouted[0] != 0:
        for i in range(NUM_ITERATIONS):
            print(f'{ripCount[i]} nets were removed in {ripTime[i + 1]:.3f} seconds')
            print(f'A* pass {i + 2} completed {aStartRouted[i + 1]}/{aStarAttempts[i + 1]} of the remaining routes in {aStarTime[i + 1]:.3f} seconds')
print(f'Router completed {100*routedNets/NET_COUNT}% of nets with a cost of {finalCost} in {(time.perf_counter() - startTime):.3f} seconds\n')  #Print % of nets that were routed and how long it took

#Check results for validity
print(f'Validating results...')
cost = 0
for i in range(NET_COUNT):
    netCost = 0
    for j in range(len(segList[i])):
        netCost = 0
        if segList[i][j][2] == 0:   #For vias
            if netList[i][6] != 0:
                cost = cost + 2
            if segList[i][j][0][0] != segList[i][j][1][0] or segList[i][j][0][1] != segList[i][j][1][1]:
                print(f'Error: Via on Net {i}, Segment{j} did not go the expected direction')
            if abs(segList[i][j][0][2] - segList[i][j][1][2]) != 1:
                print(f'Error: Via on Net {i}, Segment{j} is not the correct length')
        elif segList[i][j][2] == 1: #For horizontal segments
            if netList[i][6] != 0:
                cost = cost + abs(segList[i][j][0][0] - segList[i][j][1][0])
            if segList[i][j][0][2] != segList[i][j][1][2] or segList[i][j][0][1] != segList[i][j][1][1]:
                print(f'Error: Horizontal on Net {i}, Segment{j} did not go the expected direction')
        elif segList[i][j][2] == 2: #For vertical segments
            if netList[i][6] != 0:
                cost = cost + abs(segList[i][j][0][1] - segList[i][j][1][1])
            if segList[i][j][0][2] != segList[i][j][1][2] or segList[i][j][0][0] != segList[i][j][1][0]:
                print(f'Error: Vertical on Net {i}, Segment{j} did not go the expected direction')
        else: 
            print(f'Error: Invalid segment direction on Net {i}, Segment{j}')
        if j != len(segList[i]) - 1 and netList[i][6] != 0: #If it's not the last segment and the net is routed
            if segList[i][j][1] != segList[i][j+1][0]:      #Make sure the end of this one matches the start of the next
                print(f'End of curr seg {segList[i][j][1]} Start of next seg {segList[i][j+1][0]}')
                print(f'Error: The end of Net {i}, Segment {j} does not match the start of Segment {j + 1}')
    cost = netCost + cost
    if netList[i][6] != 0:
        posx = netList[i][0]
        posy = netList[i][1]
        posz = 0
        lastMove = 0
        while (posx != netList[i][2] or posy != netList[i][3] or posz != 0):
            if segList[i][layoutGrid[posx][posy][posz][1]][2] == 0:
                if NUM_LAYERS > posz + 1 and layoutGrid[posx][posy][posz + 1][0] == i and lastMove != 6:
                    posz = posz + 1
                    lastMove = 5
                elif 0 <= posz - 1 and layoutGrid[posx][posy][posz - 1][0] == i and lastMove != 5:
                    posz = posz - 1
                    lastMove = 6
                else:
                    print(f'Error: Unable to trace Net {i}')
                    break
            elif segList[i][layoutGrid[posx][posy][posz][1]][2] == 1:
                if GRID_SIZE > posx + 1 and layoutGrid[posx + 1][posy][posz][0] == i and lastMove != 2:
                    posx = posx + 1
                    lastMove = 1
                elif 0 <= posx - 1 and layoutGrid[posx - 1][posy][posz][0] == i and lastMove != 1:
                    posx = posx - 1
                    lastMove = 2
                else:
                    print(f'Error: Unable to trace Net {i}')
                    break
            elif segList[i][layoutGrid[posx][posy][posz][1]][2] == 2:    
                if GRID_SIZE > posy + 1 and layoutGrid[posx][posy + 1][posz][0] == i and lastMove != 4:
                    posy = posy + 1
                    lastMove = 3
                elif 0 <= posy - 1 and layoutGrid[posx][posy - 1][posz][0] == i and lastMove != 3:
                    posy = posy - 1
                    lastMove = 4
                else:
                    print(f'Error: Unable to trace Net {i}')
                    break
            else:
                print(f'Error: Unable to trace Net {i}')
                break
if cost != finalCost:
    print(f'Error: calculated cost was {finalCost}, but verrified cost was {cost}')

#Format and export results
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

#Plot results
if GENERATE_GRAPHS:
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