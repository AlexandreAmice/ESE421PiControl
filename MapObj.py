from helperCodes import tsplit
import sys
from Path import Path
from helperCodes import *
import networkx as nx

class MapObj:

    def __init__(self, mapNodes, mapSpeeds):
        """
        :param mapNodes: the path of the mapNodes file
        :param mapSpeeds: the path of the mapSpeeds file
        """
        self.mapGraph = self.genMapGraph(mapNodes, mapSpeeds) #NetworkX object representing graph of the map
        self.curPlan = ['K', 'C', 'D']
        self.turnLeft = True
        self.goalNode = None
        self.lastNode = 'A'
        self.curRoad = ('A','B')
        self.lat = 39.9509507
        self.lon =  -75.1853272

    # region ConstructorHelpers
    def genMapGraph(self, mapNodes, mapSpeeds):
        """
        Generate a dictionary mapping a start node to end nodes with a speed
        :param mapSpeeds: string path to map speeds file
        :return: Dictionary mapping node to node and then speed. Exampe: A -> B -> 50 means that going
        from A to B you can go 50
        """
        # Generate a dictionary of mapping node names to GPS coordinates
        mapGraph = nx.DiGraph()
        with open(mapNodes, 'r') as f:
            lines = f.readlines()
        lines = [x.strip() for x in lines]  # remove trailing whitespace
        for l in lines:
            words = tsplit(l, ["  ", ' ', '\t'])  # splits the word based on any whitespace delimiter
            mapGraph.add_node(words[0], attr_dict={'lat': float(words[1]), 'lon': float(words[2])})


        # Make the speedlimit dictionary
        with open(mapSpeeds, 'r') as f:
            lines = f.readlines()
        lines = [x.strip() for x in lines]
        speedlimits = {}
        for l in lines:
            words = tsplit(l, ["  ", ' ', '\t'])  # splits the word based on any whitespace delimiter
            if words[0] in speedlimits.keys():
                speedlimits[words[0]][words[1]] = float(words[2])
            else:
                speedlimits[words[0]] = {words[1]: float(words[2])}
            if words[1] in speedlimits.keys(): #map both AB direction and BA directions in one pass
                speedlimits[words[1]][words[0]] = float(words[3])
            else:
                speedlimits[words[1]] = {words[0]: float(words[3])}

        for node1 in speedlimits.keys():
            for node2 in speedlimits[node1]:
                node1lat, node1lon = self.getNodeLatLon(node1, mapGraph)
                node2lat, node2lon = self.getNodeLatLon(node2, mapGraph)
                edgeLength = sphereDist(node1lat,node1lon, node2lat, node2lon, distOnly= True)
                speed = speedlimits[node1][node2]
                if speed > 0: #only connect the edge if speed is more than 0
                    cost = edgeLength/speed
                    mapGraph.add_edge(node1, node2, attr_dict={'speedLimit': speed, 'roadLength': edgeLength,
                                                               'cost': cost})
        return mapGraph

    def getNodeLatLon(self, nodeName, curGraph = None):
        if curGraph is None:
            curGraph = self.mapGraph
        return curGraph.node[nodeName]['lat'], curGraph.node[nodeName]['lon']

    def getTurnLeft(self):
        return self.turnLeft

    def getGPSCoord(self):
       return self.lat, self.lon

    def setGPSCoord(self, lat, lon):
        self.lat = lat
        self.long = lon
    # endregion



    #TODO: get shortest path. Probably will implement in the path object

    # endregion

    def findNearestPath(self, lat, lon, posEdges = None):
        '''
        accept lat long coordinates as strings or ints and find the nearest road.
        :param lat: latitude of the desired point
        :param lon: longitude of the desired point
        :return: (nearest edge, nearest node, dist to path)
        '''
        if posEdges is None:
            posEdges = self.mapGraph.edges()
        minDist = sys.maxint
        closestPoint = 'A'
        nearestPath = None
        for edge in posEdges:
            start = edge[0]
            end = edge[1]
            startNode = float(self.mapGraph.node[start]['lat']), float(self.mapGraph.node[start]['lon'])
            endNode = float(self.mapGraph.node[end]['lat']), float(self.mapGraph.node[end]['lon'])
            curDist = distPointToLine(startNode, endNode, (lat, lon))
            if curDist < minDist:
                minDist = curDist
                nearestPath = edge
                if sphereDist(startNode[0], startNode[1], lat, lon) < sphereDist(endNode[0], endNode[1], lat, lon):
                    closestPoint = start
                else:
                    closestPoint = end
        return nearestPath, closestPoint, minDist

    def setTurnLeft(self):
        '''
        determine whether to turn left based on the different in bearing between the current path and the next path
        :return:
        '''
        if len(self.curPlan) < 3: #make sure there are 3 points
            self.turnLeft = True
            return

        firstPointlabel = self.curPlan[0]
        secondPointlabel = self.curPlan[1]
        thirdPointlabel= self.curPlan[2]
        lat1, lon1 = self.mapGraph.node[firstPointlabel]['lat'], self.mapGraph.node[firstPointlabel]['lon']
        lat2, lon2 = self.mapGraph.node[secondPointlabel]['lat'], self.mapGraph.node[secondPointlabel]['lon']
        lat3, lon3 = self.mapGraph.node[thirdPointlabel]['lat'], self.mapGraph.node[thirdPointlabel]['lon']
        d = (lat3-lat1)*(lon2-lon1)-(lon3-lon1)*(lat2-lat1)
        self.turnLeft = (d > 0)

    def setGoalNode(self, nodeName):
        '''
        pass node name as a string
        :param nodeName:
        :return:
        '''
        self.goalNode = nodeName

    def guessRoadData(self):
        '''
        Changes the value of curRoad and last Node based on the guess
        :return:
        '''
        if self.lat is None:
            self.curRoad, self.lastNode = ('A', 'B'), 'A'
        if self.lastNode is None:
            self.curRoad, self.lastNode, _ = self.findNearestPath(self.lat, self.lon)[1]
        else:
            posNext = self.mapGraph.edges(self.lastNode)
            self.curRoad, self.lastNode, _ = self.findNearestPath(self.lat, self.lon, posNext)
        return self.curRoad, self.lastNode


    def curDistToNode(self,node):
        latNode, lonNode = self.mapGraph.node[node]['lat'], self.mapGraph.node[node]['lon']
        return sphereDist(self.lat, self.lon, latNode, lonNode)

    def planPath(self, current = None, goal = None):
        if current is None:
            current = self.lastNode

        if goal is None:
            goal = self.goalNode

        self.curPlan = nx.shortest_path(self.mapGraph, current, goal, weight = 'cost')
        self.lastNode = current
        self.goalNode = goal



