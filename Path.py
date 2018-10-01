from helperCodes import sphereDist, distPointToLine
import numpy as np

class Path:
    def __init__(self, startNode, endNode, mapSpeeds):
        '''
        Gives a path object composed of direct path objects between a start and end node
        :param startNode: starting nodes stored as tuple (name, lat, long)
        :param endNode: ending node stored as tuple (name, lat, long)
        :param mapSpeeds: the map speeds data
        '''
        self.startNode = startNode
        self.endNode = endNode
        self.paths = self.getComponentPaths(startNode, endNode, mapSpeeds)

    def __repr__(self):
        return "Path " + self.getStartName() + self.getEndName()

    def __str__(self):
        return "Path " + self.getStartName() + self.getEndName()

    # region Getters
    def getStart(self):
        return self.startNode

    def getStartLat(self):
        return self.startNode[1]

    def getStartLong(self):
        return self.startNode[2]

    def getStartName(self):
        return self.startNode[0]

    def getEnd(self):
        return self.endNode

    def getEndLat(self):
        return self.endNode[1]

    def getEndLong(self):
        return self.endNode[2]

    def getEndName(self):
        return self.endNode[0]
    # endregion

    #TODO: figure out how to parse the map speed data to get the component paths. Right now only returns direct paths
    def getComponentPaths(self,startNode, endNode, mapSpeeds):
        """
        get all the component direct paths in order to create a list of direct paths to get from start to end
        :param mapData:
        :return: a list where each entry is a list of direct path objects from a start node to an end node.
        """
        return [_DirectPath(startNode, endNode, mapSpeeds[self.getStartName()][self.getEndName()])]

    def isDirectPath(self):
        """
        :return: True if the Path is composed of only one direct path
        """
        return len(self.paths) == 1

    def distToStart(self, lat, long):
        '''
        given a lat and a long finds the distance to the start node
        :param lat:
        :param long:
        :return:
        '''
        return sphereDist(lat, long, self.getStartLat(), self.getStartLong())

    def distToEnd(self, lat, long):
        '''
        given a lat and a long finds the distance to the start node
        :param lat:
        :param long:
        :return:
        '''
        return sphereDist(lat, long, self.getEndLat(), self.getEndLong())

    def isOnCenterLine(self, lat, long):
        '''
        check whether a point is on the line connecting the start and end of this path
        :param lat: lat of point to check
        :param long: long of point to check
        :return: True if on center line
        '''
        epsilon = 1e-3 #allow for rounding erros
        return  self.distToEnd(lat,long) < (self.distToStart(lat,long) + epsilon) and \
                    self.distToEnd(lat,long) > (self.distToStart(lat,long) - epsilon)

    def distToCenterLine(self, lat, long):
        '''
        returns the distance to the center line from a given lat, long
        :param lat:
        :param long:
        :return:
        '''
        pointOff = (lat, long)
        endPt1 = (self.getStartLat(), self.getStartLat())
        endPt2 = (self.getEndLat(), self.getEndLong())
        return distPointToLine(endPt1, endPt2, pointOff)


class _DirectPath:
    def __init__(self, startNode, endNode, speedLimit):
        """
        Start Nodes and End Nodes passed as tuples (name of node, lat, long)
        :param startNode: (name of node, lat, long)
        :param endNode: (name of node, lat, long)
        :param speedLimit: speed limit between start and end node
        """
        self.startNode = startNode
        self.endNode = endNode
        self.speedLimit = speedLimit

    def getStart(self):
        return self.startNode

    def getStartLat(self):
        return self.startNode[1]

    def getStartLong(self):
        return self.startNode[2]

    def getStartName(self):
        return self.startNode[0]

    def getEnd(self):
        return self.endNode

    def getEndLat(self):
        return self.endNode[1]

    def getEndLong(self):
        return self.endNode[2]

    def getEndName(self):
        return self.endNode[0]


    def distToStart(self, lat, long):
        '''
        given a lat and a long finds the distance to the start node
        :param lat:
        :param long:
        :return:
        '''
        return sphereDist(lat, long, self.getStartLat(), self.getStartLong())

    def distToEnd(self, lat, long):
        '''
        given a lat and a long finds the distance to the start node
        :param lat:
        :param long:
        :return:
        '''
        return sphereDist(lat, long, self.getEndLat(), self.getEndLong())

    def isOnCenterLine(self, lat, long):
        '''
        check whether a point is on the line connecting the start and end of this path
        :param lat: lat of point to check
        :param long: long of point to check
        :return: True if on center line
        '''
        epsilon = 1e-3 #allow for rounding erros
        return  self.distToEnd(lat,long) < (self.distToStart(lat,long) + epsilon) and \
                    self.distToEnd(lat,long) > (self.distToStart(lat,long) - epsilon)

    def distToCenterLine(self, lat, long):
        '''
        returns the distance to the center line from a given lat, long
        :param lat:
        :param long:
        :return:
        '''
        pointOff = (lat, long)
        endPt1 = (self.getStartLat(), self.getStartLat())
        endPt2 =  (self.getEndLat(), self.getEndLong())
        return distPointToLine(endPt1, endPt2, pointOff)
