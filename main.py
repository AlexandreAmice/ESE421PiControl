from MapObj import MapObj

mainMap = MapObj('mapPennParkNodes.txt', 'mapPennParkEdges.txt')
lat = 60 #input("What is your Lat?")
long = -75.1858327#input("What is your Long?")
(nearestPath, nearestNode, minDist) = mainMap.findNearestPath(float(lat), float(long))
print "The nearest path is " + str(nearestPath)
print "You probably started at " + str(nearestNode)
print "You are " + str(minDist) + " meters from the nearest path"