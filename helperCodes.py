from math import sin, cos, sqrt, atan2, radians
from numpy.linalg import norm
import numpy as np

def tsplit(s, sep):
    stack = [s]
    for char in sep:
        pieces = []
        for substr in stack:
            pieces.extend(substr.split(char))
        stack = pieces
    return stack

def sphereDist(lat1, long1, lat2, long2, distOnly = True):
    '''
    measures the distance in meters between two points give their latitude and longitude in degrees. Uses the haversine
    formula for distance. Implementation found on stack overflow at
    https://stackoverflow.com/questions/19412462/getting-distance-between-two-points-based-on-latitude-longitude
    :param lat1:
    :param long1:
    :param lat2:
    :param long2:
    :param distOnly: whether to return only the distance or the whole tuple. True -> distance only
    :return: (distance between two points, distance between the lat lines, and distance between the long lines)
    distOnly -> return only the distance
    '''
    R = 6378.137

    lat1 = radians(lat1)
    lon1 = radians(long1)
    lat2 = radians(lat2)
    lon2 = radians(long2)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = R * c
    if distOnly:
        return distance

    else:
        return (distance, dlat, dlon)


def distPointToLine(endPt1, endPt2, pointOff, distOnly = True):
    '''
    returns the distance from pointOff to line defined by endPt1, endPt2
    :param endPt1: first point of line as (x,y) tuple
    :param endPt2: second point of line (x,y) tuple
    :param pointOff: point not on line (x,y) tuple
    :param distOnly: return only the distance or the distance plus the nearest lat and nearest long
    :return: distance from pointOff to line (x,y) tuple constrained to points lying inside the end points
    '''
    #calculated using equations from https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Cartesian_coordinates
    (minX, ptX) = min((endPt1[0], endPt2[0])), (endPt1[0], endPt2[0]).index(min(endPt1[0], endPt2[0]))
    minY. pt = min(endPt1[1], endPt2[1]), (endPt1[1], endPt2[1]).index(min(endPt1[1], endPt2[1]))
    maxX = max(endPt1[0], endPt2[0]), (endPt1[0], endPt2[0]).index(max(endPt1[0], endPt2[0]))
    maxY = max(endPt1[1], endPt2[1]), (endPt1[1], endPt2[1]).index(min(endPt1[1], endPt2[1]))
    m = float(endPt1[1] - endPt2[1])/float(endPt1[0] - endPt2[0])
    a = -m
    b = 1
    c = ((-1)*endPt1[1] + m * endPt1[1])
    nearestX = float(b*(b*pointOff[0] - a*pointOff[1])- a * c)/float(a**2 + b**2) #in lat
    nearestY = float(a* ((-1)*b*pointOff[0] + a * pointOff[1])-b*c)/float(a**2 + b**2) #in long

    #constrain this to lie inside the endpoints
    if nearestX < minX:
        nearestX = minX
        if nearestY < minY:
            nearestY = minY
        elif nearestY > maxY:
            nearestY = maxY
    elif nearestX > maxX:
        nearestX = maxX
        if nearestY < minY:
            nearestY = minY
        elif nearestY > maxY:
            nearestY = maxY
    elif nearestY < minY:
        nearestY = minY
    elif nearestY > maxY:
        nearestY = maxY
    #DEPRECATED CODE AS ONLY CALCULATE LAT LONG OFFSETS
    #endPt1 = np.array(endPt1)
    #endPt2 = np.array(endPt2)
    #d = norm(np.cross(endPt2-endPt1, endPt1 - pointOff))/norm(endPt2-endPt1)

    dist = sphereDist(pointOff[0], pointOff[1], nearestX, nearestY)
    print pointOff[0], pointOff[1], nearestX, nearestY
    if distOnly:
        return dist
    else:
        return (dist, nearestX, nearestY)