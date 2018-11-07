from MapObj import MapObj
import time
import threading
import matplotlib.pyplot as plt
import networkx as nx
#from CameraTests import RoadFinderTest

curLat = 39.9509507  #dummy values will be updated in the loop TODO
curLon =  -75.1853272 #dummy values will be updated in the main loop TODO
turnLeft = True

#setup map object
pathPlan = MapObj('mapPennParkNodes.txt', 'mapPennParkEdges.txt')
currentRoad = pathPlan.guessRoadData()
count = 1*10**20
flag = True
nodeNearThresh = 0.001
while True:
    if count > 20:
        count = 0
        if flag:
            curLat, curLon = pathPlan.getNodeLatLon('C')
            pathPlan.setGoalNode('G')
            flag = not flag
            print pathPlan.goalNode
        else:
            curLat, curLon = pathPlan.getNodeLatLon('T')
            pathPlan.setGoalNode('S')
            flag = not flag
            print pathPlan.goalNode
    dist1 = pathPlan.curDistToNode(currentRoad[0][0])
    dist2 = pathPlan.curDistToNode(currentRoad[0][1])

    if not (dist1 < nodeNearThresh or dist2 < nodeNearThresh): #if we are too close to the nodes of the road let jesus take the wheel and hope car turns correctly
        pathPlan.setGPSCoord(curLat,curLon)
        guessRoadData = pathPlan.guessRoadData()
        if currentRoad[0] != guessRoadData[0]: #if the current road has changed replan the route
            currentRoad = guessRoadData
            pathPlan.planPath()
            pathPlan.setTurnLeft()
            turnLeft = pathPlan.getTurnLeft()
            print pathPlan.curPlan
            print turnLeft

    count += 1
    time.sleep(0.1)


