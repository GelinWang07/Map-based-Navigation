
#!usr/bin/python
step = 1
import cv2
import rospy
import numpy as np
from gridGeneration import *
from pathTraversal import *
from pathPlanning import *
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import copy


endGoalX = []
endGoalY = []
unreachedGoalX = []
unreachedGoalY = []
lastgoal = False

def mapClient():
    rospy.init_node('grid_exploration', anonymous= False)
    rospy.wait_for_service('dynamic_map')
    finished = False
    maybefinished = False
    nothingGenerated = False
    loopCounter = 0
    global step
    global generated_points_green
    global generated_points_red
    global lastgoal

    while not finished:
        lastgoal = False
        loopCounter += 1
        try:
            getMap = rospy.ServiceProxy('dynamic_map', GetMap)
            data = getMap().map
        except (rospy.ServiceException) as error:
            print("Service call failed: %s"%error)
            return
        
        data_array = np.array(list(data.data))
        map_height = data.info.height
        map_width = data.info.width
        data_array_reshape = np.reshape(data_array,(map_height,map_width))
        print(data_array_reshape.shape[0],data_array_reshape.shape[1])
        resolution_cpm = 1/data.info.resolution
        resolution_cpm = round(resolution_cpm)
        resolution_cpm = int(resolution_cpm)
        step = resolution_cpm
        print(data.info.origin)
        print("Map resolution mpc: " + str(data.info.resolution))
        print("Map resolution cpm: " + str(resolution_cpm))
        convertedMap = convertMap(data, data_array_reshape)
        mapImage = np.float32(convertedMap)
        kernel = np.ones((4,4),np.uint8)
        openedImage = cv2.morphologyEx(mapImage, cv2.MORPH_OPEN, kernel, iterations = 2)

        cv2.imwrite('ori'+str(loopCounter)+'.png',mapImage)
        cv2.imwrite('filter'+str(loopCounter)+'.png',openedImage)

        if loopCounter == 1:
            visitedPhysicalMap = np.zeros_like(data_array_reshape, dtype = int)
            oldGrid = np.zeros_like(data_array_reshape, dtype = int)
            startPoint = getRobotPosition(data)
            oldGridList = list()
    
        print("Generating Grid. Starting " +str(startPoint))
        oldGrid, oldGridList = generateGrid(startPoint,  openedImage, oldGrid, oldGridList, resolution_cpm)

        if len(oldGridList) == 1:
            print("Nothing generated!!! rotate once")
            rotate(20,350, True)
            if nothingGenerated:
                finished = True
            nothingGenerated = True
            continue
        else:
            nothingGenerated = False

        grid_asDouble = oldGrid.astype(float)
        grid_dil = cv2.dilate(grid_asDouble, None)
        physical_asDouble = visitedPhysicalMap.astype(float)
        phys_dil = cv2.dilate(physical_asDouble, None)
        colouredImage = cv2.cvtColor(openedImage, cv2.COLOR_GRAY2RGB)
        colouredImage[grid_dil>0.9*grid_dil.max()]=[0,0,255]
        colouredImage[phys_dil>0.9*phys_dil.max()]=[255,0,0]
        colouredImage[startPoint.y, startPoint.x] = [0,255,0]

        notVisitedList = getNotVisitedList(oldGridList, visitedPhysicalMap)

        if(len(notVisitedList) < 1):
            if maybefinished:
                finished = True
                continue
            else:
                maybefinished = True
                continue
        else:
            maybefinished = False

        pathList = getPath(notVisitedList, startPoint, step)


        if len(pathList) < 2:
            print("error, no points in pathList")
            return

        drawPointsInMap(pathList,data)

        #draw path as line
        for i in range(len(pathList)-1):
            c = (255,0,0)
            cv2.arrowedLine(colouredImage, pathList[i].getTuple(), pathList[i+1].getTuple(), c,1,8,0,0.2)

        print("Starting Path " + str(startPoint))

        colouredImage = cv2.flip(colouredImage,0)
        cv2.imwrite('path'+str(loopCounter)+'.png',colouredImage)

        #for i in range(len(gridList))::
        #    print("Generated Point: " + str(gridList[i].y) + " " +  str(gridList[i].x) + " (Y,X)")
        for i in range(1, len(pathList)):
            bestGoalInWorld = copy.deepcopy(pathList[i])
            bestGoalInWorld.visit(visitedPhysicalMap) 
            if(i == (len(pathList) - 1)):
                print("Last Goal")
                lastgoal = True
            print("Moving to Goal #" + str(i))
            if not moveToGoal(bestGoalInWorld, data, lastgoal):
                tfBuffer = tf2_ros.Buffer()
                listener = tf2_ros.TransformListener(tfBuffer)
                try:
                    trans = tfBuffer.lookup_transform('map', 'base_link' ,rospy.Time(0), rospy.Duration(10.0))
                    print("Current Position (X,Y,Z):")
                    print(trans.transform.translation)
                    print(trans.transform.rotation)
                    bestGoalInWorld.isVisited(visitedPhysicalMap)
                except tf2_ros.TransformException as ex:
                    print("error")
                    print(type(ex))

                print("Error moveToGoal returned false")
                print("Goal is: " + str(pathList[i]))
            else:
                startPoint = bestGoalInWorld
            rospy.sleep(1)

if __name__ =="__main__":
    mapClient()
