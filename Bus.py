import threading
import picamera
from picamera.array import PiRGBArray
import warnings
import time
from MapObj import MapObj
import cv2
from threading import Lock
from ArduinoCom import ArduinoCom
from Follower import findRibbon
from PathFinder2 import FindEdge
import numpy as np

print __name__
"""
This class serves as the runner for each component of the project. It maintains a list of global variables shared between
components. Each module of the code can interact with these variables to independently run their own calculations. The
primary responsibility of this class to run the threads and keep them synchronized.
"""

########################################################################################################################
#START GLOBAL DATA
########################################################################################################################
curLat = 39.9509507
curLon = -75.185327
gpsV = 0
psiD = -20
desiredOffset = 3
curOffset = 3
camLookLeft = False
lastNode = 'A'
speedD = 50
gpsPsi = 0

#LOCKS. USE CAUTION WHEN CHANGING AS THIS CAN CAUSE ERRATIC BEHAVIOR.
#This is poor code design. I need to make this more elegant.
curLatLock = Lock()
curLonLock = Lock()
psiDLock = Lock()
desiredOffsetLock = Lock()
curOffsetLock = Lock()
camLookLeftLock = Lock()
lastNodeLock = Lock()
speedDLock = Lock()
gpsVLock = Lock()
gpsPsiLock = Lock()
locks = [curLatLock, curLonLock, psiDLock, desiredOffsetLock, curOffsetLock, camLookLeftLock, lastNodeLock, speedDLock,
         gpsVLock, gpsPsiLock]

def releaseAllLocks():
    for l in locks:
        if l.locked():
            l.release()

########################################################################################################################
#END GLOBAL DATA
########################################################################################################################


########################################################################################################################
#START MAP OBJECT THREAD CLASS
########################################################################################################################
class MapThread(threading.Thread):
    """
    A runner class that maintains the Map information and Path planning for the car.
    """
    def run(self):
        print "Launching Map Thread"
        global camLookLeft, curLon, curLat, psiD, lastNode, locks
        pathPlan = MapObj('mapPennParkNodes.txt', 'mapPennParkEdges.txt')
        while True:
            try:
                tempLookLeft = pathPlan.getTurnLeft()
                camLookLeftLock.acquire()
                camLookLeft = tempLookLeft
                camLookLeftLock.release()

                if (curLat, curLon) != pathPlan.getGPSCoord():
                    pathPlan.setGPSCoord(curLat, curLon)

                nearestPath, closestPoint, minDist = pathPlan.findNearestPath(curLat,curLon)
                if nearestPath[0] in pathPlan.curPlan and nearestPath[1] in pathPlan.curPlan:
                    lastNode = pathPlan.lastNode
                if nearestPath != pathPlan.curRoad:
                    pathPlan.guessRoadData()
                    if pathPlan.lastNode:
                        pass
                    pathPlan.planPath()
            except:
                releaseAllLocks() #if there is an exception release all the locks so there is no accidental blocking

########################################################################################################################
#END MAP OBJECT THREAD CLASS
########################################################################################################################

########################################################################################################################
#START ROADEDGE THREAD CLASS
########################################################################################################################

class CameraThread(threading.Thread):
    """
    Computer vision class which attempts to find the edge of the road and determine the distance to the edge. This allows
    the car to adjust its wheels to maintain a constant distance from the edge of the road.
    """
    def run(self):
        print "Launching Cam Thread"
        global camLookLeft
        global psiD

        warnings.filterwarnings('error')

        #Set up camera. Adjust the image size to reduce computational speed at the cost of less photo information.
        camera = picamera.PiCamera()
        image_size = (960/2, 544/2)  # (16*photoHeight/9, photoHeight)
        camera.resolution = image_size
        camera.framerate = 7
        camera.vflip = False
        camera.hflip = False
        # camera.exposure_mode='off'
        rawCapture = PiRGBArray(camera, size=image_size)
        # allow the camera to warmup
        time.sleep(0.1)

        #the edge finding object
        edgeFinder = FindEdge(None,image_size[1]/2)

        print 'ready for loop'
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            #The above sets the camera in video mode and keeps a constant buffer
            try:
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                image = frame.array
                rawCapture.truncate()
                rawCapture.seek(0)
                
                edgeFinder.set_look_left(camLookLeft)

                # show the frame
                edgeFinder.set_new_image(image)
                edgeFinder.collect_des_edge()
                edge, pic = edgeFinder.draw_des_edge()
                psiD = edgeFinder.calc_phi_r()
                
                imgray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
                retr, thresh = cv2.threshold(imgray,127,255,0)
                contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                temp = cv2.drawContours(image, contours, -1, (255,0,0))
                cv2.imshow("edge", edge)
                cv2.imshow("org", pic)

                key = cv2.waitKey(1) & 0xFF

                # clear the stream in preparation for the next frame
                

                # if the `q` key was pressed, break from the loop
                if key == ord("q"):
                    break
                
            except Exception as e:
                print e
                releaseAllLocks() #if there is an error release all the locks

########################################################################################################################
#END ROADEDGE THREAD CLASS
########################################################################################################################

########################################################################################################################
#START ARDUINO COM CLASS
########################################################################################################################
class ArduinoComThread(threading.Thread):
    """
    The communication thread. This thread allows for communication between the Arduino and the Pi
    """
    def run(self):
        print "Launching Com Thread"
        global curLat, curLon, psiD, desiredOffset, curOffset, camLookLeft, lastNode, gpsV
        global curLatLock, curLonLock, psiDLock, desiredOffsetLock, curOffsetLock, camLookLeftLock, lastNodeLock, gpsVLock, locks

        sendCtr = 0 #variable that increments in order to determine which command to send to the Arduino
        receiveCtr = 0 #variable that increments in order to determine which data to get to the Arduino

        #communicating object
        com = ArduinoCom()

        while True:
            time.sleep(1)
            receiveFromArd = {'gpsLat': (curLat, curLatLock), 'gpsLon': (curLon, curLonLock), 'gpsV': (gpsV, gpsVLock), 'gpsPsi': (gpsPsi, gpsPsiLock)}
            sendToArd = {'psiD': (psiD, psiDLock), 'speedD': (speedD, speedDLock)}

            if sendCtr >= len(sendToArd):
                sendCtr = 0
            #if receiveCtr >= len(receiveFromArd):
            #    receiveCtr = 0
            try:
                #send data
                sendDataName = sendToArd.keys()[sendCtr]
                #print sendDataName
                varToSend, varToSendLock = sendToArd[sendDataName]
                varToSendLock.acquire()
                com.setData(sendDataName, varToSend)
                varToSendLock.release()
                com.sendData(sendDataName)
                
                #incr counter
                sendCtr += 1
               
                #acquire data
                
                # acqDataName = receiveFromArd.keys()[receiveCtr]
                # #print acqDataName
                # varToAcq, varToAcqLock = receiveFromArd[acqDataName]
                # com.getData(acqDataName)
                # varToAcqLock.acquire()
                # varToAcq = com.getData(acqDataName)
                # varToAcqLock.release()

                #incr counter
                #receiveCtr += 1
                
                

            except Exception as e:
                #print "failed communication"
                #print e
                releaseAllLocks()


########################################################################################################################
#END ARDUINO COM CLASS
########################################################################################################################

########################################################################################################################
#BEGIN RIBBON TRACK CLASS
########################################################################################################################
class RibbonTrackThread(threading.Thread):
    def run(self):
        print "Launching Ribbon Tracking Thread"
        global psiD
        global speed

        camera = picamera.PiCamera()
        photoHeight = 540
        image_size = (960 / 2, 544 / 2)  # (16*photoHeight/9, photoHeight)
        camera.resolution = image_size  # (960, 540)#(16*photoHeight/9, photoHeight)
        camera.framerate = 7
        camera.vflip = False
        camera.hflip = False
        # camera.exposure_mode='off'
        rawCapture = PiRGBArray(camera, size=image_size)
        # allow the camera to warmup
        time.sleep(0.1)
        ribbonFinder = findRibbon()

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            try:
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                image = frame.array
                ribbonFinder.setImage(image)
                cv2.imshow("RibbonFinder", ribbonFinder.findRib())
                key = cv2.waitKey(1) & 0xFF

                # clear the stream in preparation for the next frame
                rawCapture.truncate()
                rawCapture.seek(0)

                psiDLock.acquire()
                psiD = findRibbon.calcPsiOffset()
                psiDLock.release()

                # if the `q` key was pressed, break from the loop
                if key == ord("q"):
                    break
            except:
                releaseAllLocks()


########################################################################################################################
#END RIBBON TRACK CLASS
########################################################################################################################

if __name__ == "__main__":
    print "running main"
    comThread = ArduinoComThread()
    comThread.setDaemon(True)
    comThread.start()
    #prevents background program from running on exit

    #ribThread = RibbonTrackThread()
    #ribThread.setDaemon(True)
    #ribThread.start()

    #mapThread = MapThread()
    #mapThread.setDaemon(True)
    #mapThread.start()

    camThread = CameraThread()
    camThread.setDaemon(True)
    camThread.start()
    
    #keep main thread alive
    count = 0;
    while True:
        pass
        temp = raw_input("new speedD")
##        psiDLock.acquire()
##        psiD = temp
##        psiDLock.release()
        
        #
        
        






