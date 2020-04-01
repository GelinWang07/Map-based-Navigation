from gridGeneration import sortingForRow, myPoint
import pathTraversal
import numpy as np
import math
import sys

rotateTimes = 0
def getPath(gridList, currentPoint, step):
    #print("currentPoint:" + str(currentPoint))
    rowList = list()
    rowList.append(currentPoint)

    colList = list()
    colList.append(currentPoint)
    for i in range(len(gridList)):
        if gridList[i].hasSameY(currentPoint) and not gridList[i].isSameAs(currentPoint):
            rowList.append(gridList[i])
        if gridList[i].hasSameX(currentPoint) and not gridList[i].isSameAs(currentPoint):
            colList.append(gridList[i])

    if len(rowList) == 1 and len(colList) == 1:
        nextPath = getNextPath(gridList, currentPoint, step)
        path = list()
        path.append(currentPoint)
        if nextPath is not None:
            path.extend(nextPath)
        else:
            pathTraversal.rotate(20,350, True)
            global rotateTimes
            rotateTimes += 1
            if rotateTimes > 2:
                finished = True
        return path

    global sortingForRow
    sortingForRow = True
    rowList.sort()
    sortingForRow = False
    colList.sort()

    listList = list()

    rowListLeft = getGaplessPathLeft(rowList, currentPoint, step)
    listList.append(rowListLeft)
    rowListRight = getGaplessPathRight(rowList, currentPoint, step)
    listList.append(rowListRight)
    colListUp = getGaplessPathUp(colList, currentPoint, step)
    listList.append(colListUp)
    colListDown = getGaplessPathDown(colList, currentPoint, step)
    listList.append(colListDown)

    selectedPath = max(listList, key=len)

    if len(selectedPath) < 2:
        nextPath = getNextPath(gridList, currentPoint, step)
        selectedPath = list()
        selectedPath.append(currentPoint)
        if nextPath is not None:
            selectedPath.extend(nextPath)
        return selectedPath
    return selectedPath

def getNextPath(gridList, currentPoint, step):
    gridList.remove(currentPoint)
    nextPoint = getBestGoal(gridList, currentPoint)
    global rotateTimes
    if(nextPoint is None):
        print("No next point and rotate once")
        #TODO here to change
        return None
    return getPath(gridList, nextPoint, step)

def getGaplessPathLeft(rowList, currentPoint, step):
    sign = -1
    tmp = list()
    currentFound = False
    for i in reversed(range(0, len(rowList))):
        if currentFound:
            if tmp[-1].x + (sign*step) == rowList[i].x:
                tmp.append(rowList[i])
            else:
                return tmp
        elif rowList[i].isSameAs(currentPoint):
            currentFound = True
            tmp.append(currentPoint)
    #print("Count left: " + str(len(tmp)))
    return tmp

def getGaplessPathRight(rowList, currentPoint, step):
    sign = 1
    tmp = list()
    currentFound = False
    for i in range(0, len(rowList)):
        if currentFound:
            if tmp[-1].x + (sign*step) == rowList[i].x:
                tmp.append(rowList[i])
            else:
                return tmp
        elif(rowList[i].isSameAs(currentPoint)):
            currentFound = True
            tmp.append(currentPoint)
    #print("Count right: " + str(len(tmp)))
    return tmp

def getGaplessPathDown(colList, currentPoint, step):
    sign = -1
    tmp = list()
    currentFound = False
    for i in reversed(range(0, len(colList))):
        if currentFound:
            if tmp[-1].y + (sign*step) == colList[i].y:
                tmp.append(colList[i])
            else:
                return tmp
        elif(colList[i].isSameAs(currentPoint)):
            currentFound = True
            tmp.append(currentPoint)
    #print("Count down: " + str(len(tmp)))
    return tmp

def getGaplessPathUp(colList, currentPoint, step):
    sign = 1
    tmp = list()
    currentFound = False
    for i in range(0, len(colList)):
        if currentFound:
            if tmp[-1].y + (sign*step) == colList[i].y:
                tmp.append(colList[i])
            else:
                return tmp
        elif(colList[i].isSameAs(currentPoint)):
            currentFound = True
            tmp.append(currentPoint)
    #print("Count up: " + str(len(tmp)))
    return tmp


def getBestGoal(goals, currentPoint):
    if len(goals) == 0:
        print("No goals supplied returning none")
        return None
    bestSquareDist = sys.maxint
    for goal in goals:
        if goal.isSameAs(currentPoint):
            continue
        squareDist = np.square(goal.x-currentPoint.x) + np.square(goal.y - currentPoint.y)
        if squareDist < bestSquareDist:
            bestSquareDist = squareDist
            bestGoal = goal
    return bestGoal
