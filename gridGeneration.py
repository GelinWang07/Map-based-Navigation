import numpy as np
import rospy
from collections import deque
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


generated_points_green = 1
generated_points_red = 1
goal_points_x = []
goal_points_y = []
sortingForRow = True

def countNeighbours(gridMatrix, myPoint, resolution_cpm):
    verticalNeighbourCount = 0
    horizontalNeighbourCount = 0
    stepsize = resolution_cpm
    steps = [-1, 1]
          
    for vertical_step in steps:
        if myPoint.y+vertical_step*stepsize >= 0 and myPoint.y+vertical_step*stepsize < gridMatrix.shape[0] and gridMatrix[myPoint.y+vertical_step*stepsize][myPoint.x] == 1:
            verticalNeighbourCount += 1
     
    for horizontal_step in steps:
        if myPoint.x+horizontal_step*stepsize >= 0 and myPoint.x+horizontal_step*stepsize < gridMatrix.shape[1] and gridMatrix[myPoint.y][myPoint.x+horizontal_step*stepsize] == 1:
            horizontalNeighbourCount += 1
     
    
    return verticalNeighbourCount, horizontalNeighbourCount

def getCorners(gridMatrix, gridList, resolution_cpm):
    corners = deque()
    for currentPoint in gridList:
        verticalNeighbourCount, horizontalNeighbourCount = countNeighbours(gridMatrix, currentPoint, resolution_cpm)
        if(verticalNeighbourCount == 1 and horizontalNeighbourCount == 1):
            corners.append(currentPoint)
            print("found a corner")
        else:
            print("cannot find a corner, move to next place")
    return corners


def convertMap(data, data_array_reshape):
    tmp = np.zeros_like(data_array_reshape)
    for y in range(data.info.height):
        for x in range(data.info.width):
            if data_array_reshape[y,x]==-1:
                tmp[y,x]=0
            else:
                tmp[y,x] = 100 - data_array_reshape[y,x]
    return tmp

def generateGrid(startingPoint, mapMatrix, oldGrid, oldGridList, resolution_cpm):
    grid = oldGrid
    gridList = list()
    visitedMap = np.zeros_like(oldGrid, dtype = int)

    queue = deque()
    queue.append(startingPoint)
    startingPoint.visit(visitedMap)
    for i in range(len(oldGridList)):
        if not oldGridList[i].isSameAs(startingPoint):
            oldGridList[i].visit(visitedMap)
            queue.append(oldGridList[i])
    while queue:
        point = queue.popleft()
        gridList.append(point)
        grid[point.y,point.x] = 1
        neighbours = point.generateNeighbours(mapMatrix, visitedMap, resolution_cpm)
        for n in neighbours:
                #print("Generated Point: " + str(n.y) + " " +  str(n.x) + " (Y,X)")
                queue.append(n)
    return grid, gridList

def getNotVisitedList(gridList, visitedPhysicalMap):
    # range (1, len) because startingPoint should always be kept
    notVisitedList = deque()
    notVisitedList.append(gridList[0])
    for i in range(1, len(gridList)):
        if not gridList[i].isVisited(visitedPhysicalMap):
            notVisitedList.append(gridList[i])
    return notVisitedList

def checkDist(mapMatrix,radInMeter, rowNumber, columnNumber,resolution_cpm):
    radInPix = int(radInMeter * resolution_cpm)
    return [[mapMatrix[i][j] if  i >= 0 and i < len(mapMatrix) and j >= 0 and j < len(mapMatrix[0]) else 200
            for j in range(columnNumber-1-radInPix, columnNumber+radInPix)]
                for i in range(rowNumber-1-radInPix, rowNumber+radInPix)] 

def drawGoalsInRviz(goalx,goaly,color,id):
    markerArrayPub = rospy.Publisher('/visualization_marker_array', MarkerArray,queue_size=1)
    markerArray = MarkerArray()
    marker = Marker()
    if(color[1]==1.0):
        rate = rospy.Rate(0.2)
        rate.sleep()
    for i in range(len(goalx)):
        marker.header.frame_id = "/odom"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = goalx[i]
        marker.pose.position.y = goaly[i]
        marker.pose.position.z = 0
    
        markerArray.markers.append(marker)

        # Renumber the marker IDs
        #id = 0
        for m in markerArray.markers:
            m.id = id[i]

        # Publish the MarkerArray
        markerArrayPub.publish(markerArray)
        rospy.sleep(0.01) 

def drawPointsInMap(pathList,data):
    global goal_points_x
    global goal_points_y
    global generated_points_green
    print(len(pathList),"pathList")
    for i in range(1,len(pathList)):
        cell_currentPosX, cell_currentPosY = pathList[i].x,pathList[i].y
        currentX = (cell_currentPosX * data.info.resolution)  + data.info.origin.position.x
        currentY = (cell_currentPosY * data.info.resolution)  + data.info.origin.position.y
        goal_points_x.append(currentX)
        goal_points_y.append(currentY)
        print(np.arange(generated_points_green,len(pathList) + generated_points_green))
    drawGoalsInRviz(goal_points_x,goal_points_y,[0.0,1.0,0.0],np.arange(generated_points_green,len(pathList) + generated_points_green))
    generated_points_green += len(pathList)
    goal_points_x = []
    goal_points_y = []

class myPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def visit(self, visitedMap):
        visitedMap[self.y,self.x] = 1
        return visitedMap

    def unvisit(self, visitedMap):
        visitedMap[self.y,self.x] = 0
        return visitedMap
    
    def isVisited(self, visitedMap):
        try:
            return visitedMap[self.y,self.x] > 0
        except IndexError:
            return True

    def isFree(self, mapMatrix):
        if(self.x < 0 or self.y < 0):
            return False
        try:
            return mapMatrix[self.y,self.x] > 90
        except IndexError:
            return False

    def generateNeighbours(self, mapMatrix, visitedMap,resolution_cpm):
        #print("Generating on Point: " + str(self.y) + " " +  str(self.x) + " (Y,X)")
        neighbours = []
        stepdirections = np.array([[-1,0],[1,0],[0,-1],[0,1]])
        stepsize = resolution_cpm       
        
        for step in stepdirections:
            n = myPoint(self.x+step[0]*stepsize, self.y+step[1]*stepsize)
            if(n.isFree(mapMatrix) and not n.isVisited(visitedMap)):

                test_matrix = checkDist(mapMatrix,0.5,self.y+step[1]*stepsize,self.x+step[0]*stepsize,resolution_cpm)
                t_matrix = np.asarray(test_matrix)
                if 0 not in t_matrix:
                    n.visit(visitedMap)
                    neighbours.append(n)                    
                else:
                    pass
                    #print("Point: " + str(n.y) + " " +  str(n.x) + " (Y,X) near Wall or Border")
            else:
                pass                
                #print("Point: " + str(n.y) + " " +  str(n.x) + " (Y,X) not free or already visited")
        return neighbours

    def __str__(self):
        return "Point (X,Y): (" + str(self.x) + "," + str(self.y) + ")"

    def getTuple(self):
        return (self.x, self.y)

    def isSameAs(self, point):
        return self.hasSameX(point) and self.hasSameY(point)

    def hasSameX(self, point):
        return self.x == point.x 

    def hasSameY(self, point):
        return self.y == point.y

    def __lt__(self, b):
        global sortingForRow
        if sortingForRow:
            return self.x < b.x
        else:
            return self.y < b.y
