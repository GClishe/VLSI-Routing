#
#
#
#

#User parameters
dataName = 'netlist_100_100'         #Name of netlist file. Make sure original folder names are used and that result folders exist
masterSeed = 123456789          #Set seed to make RND reproducable.
numLayers = 9

#Set import / export folder names based on data set
if dataName[:2] == 'Re':                #If the name of the data set starts with Re
    folderName = 'Reval_netlists-2'     #Assume it's Reval and set it to that folder
else:                                   #Otherwise assume Rtest. Use else here to accomadate the one called netlist_100_100
    folderName = 'Rtest_netlists'       #Set folder to Rtest

#Import libraries
import numpy as np
import pprint
import time

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
                  0,                                            #[5]Locked (0=unlcoked, 1=locked)
                  0]                                            #[6]Final cost (0=unrouted)
segList = [0] * netCount   #Create an empty list filled with a lists for each nets (normal python list for resizability) segList[net][segment][0:start, 1:end, 2:type(0=Via, 1=Hor, 2=Vert)][0:x, 1:y, 2:layer]
layoutGrid = np.full((gridSize, gridSize, numLayers, 5), -1, dtype = np.int32) #Create 3D array for layout and give each cell a few variables ([0]what net is on this cell, [1]what segment of net(prioratize start of next for overlap), [2]calculated cost from here to destination, [3]actual cost start to here, [4]net currently attempting route on cell


#Functions
def netCostStartToCell (x: int, y: int, z:int, net: int) -> int:  #Find the actual cost of a net up until a given cell
    #Look at the layoutGrid location to get net and segment (use net currently searching variable in layoutGrid)
    #Follow that segment back to the start (segments can only go in one direction), go to prior segment, repeat
    cost = 1
    return cost

def netCostCellToEnd(x: int, y: int, z:int, net:int) -> int:    #Find the predicted remaining cost of the net from this cell
    cost = abs(netList[net][2] - x) + abs(netList[net][3] - y) + z*2  #3D Manhattan z*2 for vias is fine for now
    return cost

def addVia (net: int, x:int, y:int, startLayer:int, endLayer:int):     #Adds a via segment to the segment list and layout grid. Updates cost
    global segList
    seg = len(segList[net])-1
    segList[net].insert(seg, [[x, y, startLayer], [x, y, endLayer], 0])  #Put new segment before end
    global layoutGrid
    layoutGrid[x][y][startLayer] = [net, seg, netCostCellToEnd(x,y, startLayer, net), netCostStartToCell(x, y, startLayer, net), net]  #Fill in starting layer
    layoutGrid[x][y][endLayer] = [net, seg, netCostCellToEnd(x, y, endLayer, net), netCostStartToCell(x, y, endLayer, net), net]       #Fill in destination layer
    layoutGrid[netList[net][2]][netList[net][3]][0][1] = seg + 1    #Update lower end via's segment ID on the layout grid
    layoutGrid[netList[net][2]][netList[net][3]][1][1] = seg + 1    #Update the segment ID of the upper part of the end via. This can overwrite the end of this via but the end takes priority
    if (x, y, endLayer) == (netList[net][2], netList[net][3], 1):   #If this via and the end via overlap
        cost = netCostStartToCell(netList[net][2], netList[net][3], 0, net)
        layoutGrid[netList[net][2]][netList[net][3]][0][3] =  cost  #Update cost start to cell of lower via
        netList[net][6] = cost                                      #Use this cost to update netList

def addHori (net: int, xStart:int, xEnd:int, y:int, z:int):     #Adds a horizontal segment to the segment list and layout grid. Updates cost
    global segList
    seg = len(segList[net])-1
    segList[net].insert(seg, [[xStart, y, z], [xEnd, y, z], 1])  #Put new segment before end
    global layoutGrid
    for x in range(xStart, xEnd + 1):   #Fill in cells
        layoutGrid[x][y][z] = [net, seg, netCostCellToEnd(x, y, z, net), netCostStartToCell(x, y, z, net), net]
    layoutGrid[netList[net][2]][netList[net][3]][0][1] = seg + 1    #Update lower end via's segment ID on the layout grid
    layoutGrid[netList[net][2]][netList[net][3]][1][1] = seg + 1    #Update the segment ID of the upper part of the end via. This can overwrite the end of this segment but the end takes priority
    if (xEnd, y, z) == (netList[net][2], netList[net][3], 1):       #If this segment and the end via overlap
        cost = netCostStartToCell(netList[net][2], netList[net][3], 0, net)
        layoutGrid[netList[net][2]][netList[net][3]][0][3] =  cost  #Update cost start to cell of lower via
        netList[net][6] = cost                                      #Use this cost to update netList

def addVert (net: int, x:int, yStart: int, yEnd:int, z:int):     #Adds a horizontal segment to the segment list and layout grid. Updates cost
    global segList
    seg = len(segList[net])-1
    segList[net].insert(seg, [[x, yStart, z], [x, yEnd, z], 1])  #Put new segment before end
    global layoutGrid
    for y in range(yStart, yEnd + 1):   #Fill in cells
        layoutGrid[x][y][z] = [net, seg, netCostCellToEnd(x, y, z, net), netCostStartToCell(x, y, z, net), net]
    layoutGrid[netList[net][2]][netList[net][3]][0][1] = seg + 1    #Update lower end via's segment ID on the layout grid
    layoutGrid[netList[net][2]][netList[net][3]][1][1] = seg + 1    #Update the segment ID of the upper part of the end via. This can overwrite the end of this segment but the end takes priority
    if (x, yEnd, z) == (netList[net][2], netList[net][3], 1):       #If this segment and the end via overlap
        cost = netCostStartToCell(netList[net][2], netList[net][3], 0, net)
        layoutGrid[netList[net][2]][netList[net][3]][0][3] =  cost  #Update cost start to cell of lower via
        netList[net][6] = cost                                      #Use this cost to update netList



#Start routing
startTime = time.perf_counter()     #Start timer
for i in range(netCount):  #Fill the empty lists with first and last segment as well as a value to indicate type of segment
    segList[i] = [[[netList[i][0], netList[i][1], 0], [netList[i][0], netList[i][1], 1], 0], #First via from M1 to M2
                  [[netList[i][2], netList[i][3], 1], [netList[i][2], netList[i][3], 0], 0]] #Last via from M2 to M1 
for i in range(netCount):  #Manually add start and end vias for each net to layoutGrid
    layoutGrid[segList[i][0][0][0]][segList[i][0][0][1]][0] = [i, 0, netCostCellToEnd(segList[i][0][0][0], segList[i][0][0][1], 0, i), netCostStartToCell(segList[i][0][0][0], segList[i][0][0][1], 0, i), i]
    layoutGrid[segList[i][0][0][0]][segList[i][0][0][1]][1] = [i, 0, netCostCellToEnd(segList[i][0][0][0], segList[i][0][0][1], 1, i), netCostStartToCell(segList[i][0][0][0], segList[i][0][0][1], 1, i), i]
    layoutGrid[segList[i][1][0][0]][segList[i][1][0][1]][0] = [i, 1, netCostCellToEnd(segList[i][1][0][0], segList[i][1][0][1], 0, i), -1, i]
    layoutGrid[segList[i][1][0][0]][segList[i][1][0][1]][1] = [i, 1, netCostCellToEnd(segList[i][1][0][0], segList[i][1][0][1], 1, i), -1, i]

#addVia(11, 12, 0, 1, 2)
#addVert(11, 12, 0, 99, 2)
#addVia(11,12,99,2,1)
#addHori(11,2,12,99,1)


#start with patern router for shorts. Try all the orientations for that length (line, L, and 7. Z might be too much) on M2 and M3 (lines on both, L/7 span both layers as is)
#Stat with length of 1 and lock any nets you can complete with this on M2. During the search, note the next lowest HPWL
#On the next size step, you'll need to verify that nothing is obstructing the path
#After shorts are done, do for longs. A* what's left. If I add rip up later, don't remove M1/M2 vias
#Make sure to update end via segment ID in layout grid when adding segments
#Can't trace backwards, the last segment placed must be one that overlaps with the end via



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