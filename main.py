from MapObj import MapObj

mainMap = MapObj('mapPennParkNodes.txt', 'mapPennParkEdges.txt')
print mainMap.getPathsFromTo('A', 'C')