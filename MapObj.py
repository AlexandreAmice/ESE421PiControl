from helperCodes import tsplit
import sys
from Path import Path

class MapObj:

    def __init__(self, mapNodes, mapSpeeds):
        """
        :param mapNodes: the path of the mapNodes file
        :param mapSpeeds: the path of the mapSpeeds file
        """
        self.mapNodes = self.genMapNodes(mapNodes) #dictionary mapping map nodes to their GPS coordinates
        self.mapSpeeds = self.genMapSpeeds(mapSpeeds) #dictionary mapping a start node to a dictionary mapping end nodes to speed limits
        self.mapPaths = self.genMapPaths() #dictionary mapping (start, end) tuples to Path objects


    # region ConstructorHelpers
    def genMapNodes(self, mapNodes):
        """
        Generate a dictionary mapping nodes to coordinates
        :param mapNodes: string file path of the map nodes file
        :return: the dictionary mapping map nodes to tuples of lat, long coordinates
        """

        # Generate a dictionary of mapping node names to GPS coordinates
        with open(mapNodes, 'r') as f:
            lines = f.readlines()
        lines = [x.strip() for x in lines]  # remove trailing whitespace
        nodes = {}  # dictionary mapping node names to GPS coordinates
        for l in lines:
            words = tsplit(l, ["  ", ' ', '\t'])  # splits the word based on any whitespace delimiter
            nodes[words[0]] = (float(words[1]), float(words[2]))

        return nodes

    def genMapSpeeds(self, mapSpeeds):
        """
        Generate a dictionary mapping a start node to end nodes with a speed
        :param mapSpeeds: string path to map speeds file
        :return: Dictionary mapping node to node and then speed. Exampe: A -> B -> 50 means that going
        from A to B you can go 50
        """
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

        return speedlimits

    def genMapPaths(self):
        '''
        map all the nodes to path objects
        :return: a dictionary mapping node tuples to a path object
        '''
        paths = {}
        for nodeName1 in self.mapSpeeds.keys():
            for nodeName2 in self.mapSpeeds[nodeName1]:
                node1 = (nodeName1, self.mapNodes[nodeName1][0], self.mapNodes[nodeName1][1])
                node2 = (nodeName2, self.mapNodes[nodeName2][0], self.mapNodes[nodeName2][1])
                paths[nodeName1, nodeName2] = Path(node1, node2, self.mapSpeeds)
        return paths
    # endregion


    # region Getters
    def getAllNodes(self):
        '''
        :return: whole dictionary mapping nodes to coordinates
        '''
        return self.mapNodes

    def getAllSpeeds(self):
        '''
        :return: whole dictionary mapping roads to speeds
        '''
        return self.mapSpeeds

    def getNodeCoord(self, node):
        '''
        get the coordinate of a given node. Raises Value Error of Node doesn't exist in the dictionary
        :param node: the string name of a map node
        :return: (lat, long) tuple of coordinates of the map nodes
        '''
        if node not in self.mapNodes.keys():
            raise ValueError('could not find node %s in map nodes', node)

        else:
            return self.mapNodes[node]

    def getRoadSpeed(self, start, end):
        '''
        Get the speed limit of the road connecting the start to the end road. Raises value errors if roads or nodes don't
        exist
        :param start: start direction node
        :param end: end direction node
        :return: a value giving the permissible speed limit on a road
        '''
        if start not in self.mapSpeeds.keys():
            raise ValueError('Start node %s does not exist', start)

        if end not in self.mapSpeeds.keys():
            raise ValueError('End node %s does not exist', end)

        if end not in self.mapSpeeds[start]:
            raise ValueError('Road %s%s does not exist', start, end)

        return self.mapSpeeds[start][end]

    def getAllPaths(self):
        return self.mapPaths

    def getPathsFromTo(self, nodeName1, nodeName2):
        '''
        get the from nodeName1 to nodeName2
        :param nodeName1: start node
        :param nodeName2: end node
        :return: paths from nodeName1 to nodeName2
        '''
        return self.mapPaths[(nodeName1, nodeName2)]

    #TODO: get shortest path. Probably will implement in the path object

    # endregion

    def findNearestPath(self, lat, long):
        '''
        accept lat long coordinates as strings or ints and find the nearest road.
        :param lat: latitude of the desired point
        :param long: longitude of the desired point
        :return: (nearest path, nearest node, dist to path)
        '''
        minDist = sys.maxint
        closestPoint = 'A'
        nearestPath = None
        for path in self.mapPaths.values():
            if path.isDirectPath():
                dist = path.distToCenterLine(lat, long)
                print dist
                print
                if dist < minDist:
                    minDist = dist
                    if path.distToEnd(lat,long) < path.distToStart(lat,long):
                        closestPoint = path.getEnd()
                    else:
                        closestPoint = path.getStart()
                    nearestPath = path

        return (nearestPath, closestPoint, minDist)


