from helperCodes import tsplit

#Generate a dictionary of mapping node names to GPS coordinates
with open('mapPennParkNodes.txt', 'r') as f:
    lines = f.readlines()
lines = [x.strip() for x in lines] #remove trailing whitespace
nodes = {} #dictionary mapping node names to GPS coordinates
for l in lines:
    words = tsplit(l, ["  ", ' ', '\t']) #splits the word based on any whitespace delimiter
    nodes[words[0]] = (words[1], words[2])

del lines #remove the lines variable as we no longer need it and want to overwrite it

#Make the speedlimit dictionary
with open('mapPennParkEdges.txt', 'r') as f:
    lines = f.readlines()
lines = [x.strip() for x in lines]
speedlimits = {}
for l in lines:
    words = tsplit(l, ["  ", ' ', '\t']) #splits the word based on any whitespace delimiter
    if words[0] in speedlimits.keys():
        speedlimits[words[0]][words[1]] = words[2]
    else:
        speedlimits[words[0]] = {words[1]: words[2]}
    if words[1] in speedlimits.keys():
        speedlimits[words[1]][words[0]] = words[3]
    else:
        speedlimits[words[1]] = {words[0]: words[3]}

print 'done'