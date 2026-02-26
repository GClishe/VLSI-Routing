#
#
#
#

#User parameters
dataName = 'Rtest_1500_50000'        #Name of netlist file. Make sure original folder names are used and that result folders exist
masterSeed = 123456789              #Set seed to make RND reproducable.
numLayers = 9                       #Set the number of layers available
maxPatternSize = -1                 #Set the maximum length pattern routing should be used for. Set to 0 to disable pattern routing step, and -1 for auto

#Set import / export folder names based on data set
if dataName[:2] == 'Re':                #If the name of the data set starts with Re
    folderName = 'Reval_netlists-2'     #Assume it's Reval and set it to that folder
else:                                   #Otherwise assume Rtest. Use else here to accomadate the one called netlist_100_100
    folderName = 'Rtest_netlists'       #Set folder to Rtest

#Import libraries
import numpy as np
import pprint
import time

#Set RNG Seed
rng = np.random.default_rng(masterSeed)

#Format data from the library file to be easier to deal with
data = getattr(__import__(f'{folderName}.{dataName}', fromlist = ['data']), 'data')     #Import net list data
gridSize = data['grid_size']        #Extract grid size
netCount = len(data['nets'])        #Extract number of nets
netList = np.zeros((netCount, 7), dtype = np.int32)             #Create an empty array with 7 entries per net
for i in range(netCount):                                       #Fill the empty list with data about each net
    netList[i] = [int(data['nets'][f'NET_{i}']['pins'][0][0]),  #[0]Pin0X
                  int(data['nets'][f'NET_{i}']['pins'][0][1]),  #[1]Pin0Y
                  int(data['nets'][f'NET_{i}']['pins'][1][0]),  #[2]Pin1X
                  int(data['nets'][f'NET_{i}']['pins'][1][1]),  #[3]Pin1Y
                  int(data['nets'][f'NET_{i}']['length']),      #[4]HPWL
                  0,                                            #[5]Locked (0=unlocked, 1=locked)
                  0]                                            #[6]Final cost (0=unrouted)
segList = [0] * netCount   #Create an empty list filled with a lists for each nets (normal python list for resizability) segList[net][segment][0:start, 1:end, 2:type(0=Via, 1=Hor, 2=Vert)][0:x, 1:y, 2:layer]
layoutGrid = np.full((gridSize, gridSize, numLayers, 5), -1, dtype = np.int32) #Create 3D array for layout and give each cell a few variables ([0]what net is on this cell, [1]what segment of net(prioratize start of next for overlap), [2]calculated cost from here to destination, [3]actual cost start to here, [4]net currently attempting route on cell


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
        if (abs(z - dest) != 1):    #Check that via is only one long
            print('Error: Attempted to place illegal via')
            return False
        if  dest < z:
            stepIteration = -1
        for step in range(z, dest + stepIteration, stepIteration):
            if (layoutGrid[x][y][int(step)][0] not in {-1, net}):
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
for i in range(netCount):  #Fill the empty lists with first and last segment as well as a value to indicate type of segment
    segList[i] = [[[netList[i][0], netList[i][1], 0], [netList[i][0], netList[i][1], 1], 0], #First via from M1 to M2
                  [[netList[i][2], netList[i][3], 1], [netList[i][2], netList[i][3], 0], 0]] #Last via from M2 to M1 
for i in range(netCount):  #Manually add start and end vias for each net to layoutGrid
    layoutGrid[segList[i][0][0][0]][segList[i][0][0][1]][0] = [i, 0, netCostCellToEnd(segList[i][0][0][0], segList[i][0][0][1], 0, i), netCostStartToCell(segList[i][0][0][0], segList[i][0][0][1], 0, i, 0), i]
    layoutGrid[segList[i][0][0][0]][segList[i][0][0][1]][1] = [i, 0, netCostCellToEnd(segList[i][0][0][0], segList[i][0][0][1], 1, i), netCostStartToCell(segList[i][0][0][0], segList[i][0][0][1], 1, i, 0), i]
    layoutGrid[segList[i][1][0][0]][segList[i][1][0][1]][0] = [i, 1, netCostCellToEnd(segList[i][1][0][0], segList[i][1][0][1], 0, i), -1, i]
    layoutGrid[segList[i][1][0][0]][segList[i][1][0][1]][1] = [i, 1, netCostCellToEnd(segList[i][1][0][0], segList[i][1][0][1], 1, i), -1, i]

#Attempt line / L pattern match on layers M2-M5. Reserve M6-M9 for advanced routing. Lock any 1 length routes on M2 and M3, those are optimal
if maxPatternSize != 0:     #Skip pattern routing is max size is 0
    patternLength = 1       #Set initial max HPWL size to pattern route as 1
    maxSizeTrack = 0        #Initialize temporary value for tracking largest HPWL
    if maxPatternSize == -1:    #If max pattern size is adaptive
        for i in range(netCount):   #Iterate throught the nets
            if maxPatternSize < netList[i][4]:   #And find the largest HPWL
                maxPatternSize = netList[i][4]    #Update value
    while(1):
        nextSmallestLength = gridSize * 2   #Reset next smallest tracker
        for i in range(netCount):   #For all nets
            if netList[i][6] == 0:  #If the net is unrouted
                if netList[i][4] <= patternLength:          #If the HPWL of this net is small enough
                    if netList[i][0] == netList[i][2]:      #If the start and end are on the same x
                        if segOpen(i, netList[i][0], netList[i][1], 1, netList[i][3], 2):   #Check vertical line from start to end on M2
                            print(f'Routed Net {i} with HPWL {patternLength} using a vertical line on M2')
                            addVert(i, netList[i][0], netList[i][1], netList[i][3], 1) 
                        elif segOpen(i, netList[i][0], netList[i][1], 1, 2, 0) and segOpen(i, netList[i][0], netList[i][1], 2, 3, 0) and segOpen(i, netList[i][2], netList[i][3], 3, 2, 0) and segOpen(i, netList[i][2], netList[i][3], 2, 1, 0): #If M2 vertical line fails, Check via from M2 to M3 and M3 to M4 at start and M4 to M3 and M3 to M2 at end
                            if segOpen(i, netList[i][0], netList[i][1], 3, netList[i][3], 2):  #Check vertical line on M4
                                print(f'Routed Net {i} with HPWL {patternLength} using a vertical line on M4')
                                addVia(i, netList[i][0], netList[i][1], 1, 2)
                                addVia(i, netList[i][0], netList[i][1], 2, 3)
                                addVert(i, netList[i][0], netList[i][1], netList[i][3], 3) 
                                addVia(i, netList[i][2], netList[i][3], 3, 2)
                                addVia(i, netList[i][2], netList[i][3], 2, 1)
                        #maybe try X spaces to left/right U routes
                    elif netList[i][1] == netList[i][3]:    #If the start and end are on the same y
                        if segOpen(i, netList[i][0], netList[i][1], 1, 2, 0) and segOpen(i, netList[i][2], netList[i][3], 2, 1, 0): #Check via at start from M2 to M3 and via at end from M3 to M2
                            if segOpen(i, netList[i][0], netList[i][1], 2, netList[i][2], 1):  #Try M3 horizontal line from start to end
                                print(f'Routed Net {i} with HPWL {patternLength} using a horizontal line on M3')
                                addVia(i, netList[i][0], netList[i][1], 1, 2)
                                addHori(i, netList[i][0], netList[i][2], netList[i][1], 2) 
                                addVia(i, netList[i][2], netList[i][3], 2, 1)
                            elif segOpen(i, netList[i][0], netList[i][1], 2, 3, 0) and segOpen(i, netList[i][2], netList[i][3], 3, 2, 0) and segOpen(i, netList[i][0], netList[i][1], 3, 4, 0) and segOpen(i, netList[i][2], netList[i][3], 4, 3, 0): #If M3 horizontal line fails, Check vias at start from M3 to M4 and M4 to M5, and end from M5 to M4 and M4 to M3
                                if segOpen(i, netList[i][0], netList[i][1], 4, netList[i][2], 1):  #Check M3 horizontal line from start to end on M5
                                    print(f'Routed Net {i} with HPWL {patternLength} using a horizontal line on M5')
                                    addVia(i, netList[i][0], netList[i][1], 1, 2)
                                    addVia(i, netList[i][0], netList[i][1], 2, 3)
                                    addVia(i, netList[i][0], netList[i][1], 3, 4)
                                    addHori(i, netList[i][0], netList[i][2], netList[i][1], 4) 
                                    addVia(i, netList[i][2], netList[i][3], 4, 3)
                                    addVia(i, netList[i][2], netList[i][3], 3, 2)
                                    addVia(i, netList[i][2], netList[i][3], 2, 1)
                        #maybe try X spaces to left/right U routes
                    else:                                   #Start and End point don't share x or y axis
                        if segOpen(i, netList[i][0], netList[i][1], 1, netList[i][3], 2) and segOpen(i, netList[i][0], netList[i][3], 2, 3, 0) and segOpen(i, netList[i][2], netList[i][3], 3, 2, 0): #Check the vertical path on M2 to intersection, and via from M2 to M3 at intersection, and via on end point from M3 to M2
                            if segOpen(i, netList[i][0], netList[i][3], 2, netList[i][2], 1): #Check horizontal path from intersection to end on M3
                                print(f'Routed Net {i} with HPWL {patternLength} using an L pattern on M2 and M3')
                                addVert(i, netList[i][0], netList[i][1], netList[i][3], 1)
                                addVia(i, netList[i][0], netList[i][3], 1, 2)
                                addHori(i, netList[i][0], netList[i][2], netList[i][3], 2)
                                addVia(i, netList[i][2], netList[i][3], 2, 1)
                            #elif #Check via from M3 to M4 at intersection, and via from M4 to M5 at intersection, and horizontal path from intersection to end on M5, and via from M5 to M4 at end, and via from M4 to M3 at end
                                #Route
                        #elif #Check the via from M2 to M3 at start, and via from M3 to M4 at start, and vertical path from start to intersection on M4, and via at end from M3 to M2
                            #if #Check via at intersetion from M4 to M3, and horizontal path from midpoint to end on M3
                                #Route
                            #elif #Check the via at intersection from M4 to M5, and horizontal path from midpoint to end on M5, and via at end from M5 to M4, and via at end from M4 to M3
                                #Route
                        #elif #lower hori then lower/ upper vert
                        #elif #upper hori then lower/upper vert
                        #Yeah I'm not doing z routes... unless... nah nevermind... weelllllll
            if netList[i][4] < nextSmallestLength and netList[i][4] > patternLength:  #Every iteration scan each net to find the next smallest value
                    nextSmallestLength = netList[i][4]
        if nextSmallestLength == patternLength: #If no next smallest was found  
            break   #End pattern routing
        elif nextSmallestLength > maxPatternSize:  #If the next smallest pattern size exceeds the mad pattern size
            break   #End pattern routing   
        patternLength = nextSmallestLength  #If the loop hasn't broken, commit nextSmallestLength to be the next attempted pattern length





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
for i in range(netCount):
    if netList[i][6] != 0:
        routedNets = routedNets + 1

print(f'Routed {100*routedNets/netCount}% of nets in {(time.perf_counter() - startTime):.3f} seconds')  #Print % of nets that were routed and how long it took
print(f"Exporting reults to ./Lowry_Clishe_{folderName}/{dataName}.py ...")

output = {                              #Create output data structure, add grid size and final cost
    'meta': {
        'grid_size': gridSize,
        'layer_directions': {
        },
        'total_cost': sum(entry[6] for entry in netList)
    },
    'nets': {
    }
}

for i in range(1, numLayers):           #Populate layer information
    if i % 2 == 0:
        output['meta']['layer_directions'][f'M{i + 1}'] = 'H'
    else:
        output['meta']['layer_directions'][f'M{i + 1}'] = 'V'

for i in range(netCount):               #Populate net information
    output['nets'][f'NET_{i}'] = {
        'cost': netList[i][6], 
        'pins': [(netList[i][0], netList[i][1]), (netList[i][2], netList[i][3])],
        'segments':[]
        }
    for j in range(len(segList[i])):    #Populate segments
        output['nets'][f'NET_{i}']['segments'].append({'end': (segList[i][j][1][0], segList[i][j][1][1], f'M{1 + segList[i][j][1][2]}'), 'start': (segList[i][j][0][0], segList[i][j][0][1], f'M{1 + segList[i][j][0][2]}')})

output_content = "data = " + pprint.PrettyPrinter(indent=4).pformat(output)             # Format output data
with open(f'Lowry_Clishe_{folderName}/{dataName}.py', 'w') as f:                        # Open export location
    f.write(output_content)                                                             # Export data

#Plot
print('Graphing results...')
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

fig = plt.figure()              #Initialize plot
ax = plt.axes(projection='3d')  #Set it to 3D
ax.set_zlim([1,9])              #Show M1-M9 on z axis
ax.set_xlim([0, gridSize])      #Limit to show grid
ax.set_ylim([0, gridSize])
colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan'] #List of line colors
for i in range(netCount):       
    for j in range(len(segList[i])):    #Plot each segment of each net. Use a set color for the whole net
        ax.plot(np.linspace(segList[i][j][0][0], segList[i][j][1][0]), np.linspace(segList[i][j][0][1], segList[i][j][1][1]), np.linspace(segList[i][j][0][2], segList[i][j][1][2]) + 1, color=colors[i%10], label=f'Net {i}, Seg {j}')
plt.savefig(f"Lowry_Clishe_{folderName}/{dataName}.png", dpi=300)   #Save plot
plt.show()                      #Show plot