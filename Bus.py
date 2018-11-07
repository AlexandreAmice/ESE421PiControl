import threading
import picamera
from picamera.array import PiRGBArray
import PathFinder
import warnings
import time
from MapObj import MapObj
import cv2
from thread import Lock
from ArduinoCom import ArduinoCom

########################################################################################################################
#START GLOBAL DATA
########################################################################################################################
curLat = 39.9509507
curLon = -75.185327
gpsV = 0
phiD = 0
desiredOffset = 3
curOffset = 3
camLookLeft = True
lastNode = 'A'
speedD = 50
gpsPsi = 0

#LOCKS. USE CAUTION WHEN CHANGING AS THIS CAN CAUSE ERRATIC BEHAVIOR.
#This is gross. need to consider refactor
curLatLock = Lock()
curLonLock = Lock()
phiDLock = Lock()
desiredOffsetLock = Lock()
curOffsetLock = Lock()
camLookLeftLock = Lock()
lastNodeLock = Lock()
speedDLock = Lock()
gpsVLock = Lock()
gpsPsiLock = Lock()
locks = [curLatLock, curLonLock, phiDLock, desiredOffsetLock, curOffsetLock, camLookLeftLock, lastNodeLock, speedDLock,
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
    def run(self):
        global camLookLeft, curLon, curLat, phiD, lastNode, locks
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
#START CAMERA THREAD CLASS
########################################################################################################################
class CameraThread(threading.Thread):
    def run(self):
        global camLookLeft
        global phiD

        warnings.filterwarnings('error')

        image_size = (320, 192)
        camera = picamera.PiCamera()
        camera.resolution = image_size
        camera.framerate = 7
        camera.vflip = False
        camera.hflip = False
        # camera.exposure_mode='off'
        rawCapture = PiRGBArray(camera, size=image_size)

        # allow the camera to warmup
        time.sleep(0.1)

        pathFinder = PathFinder.PathFinder(None, 100)

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            try:
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                pathFinder.setLookLeft(camLookLeft)
                image = frame.array

                # show the frame
                # lines.project_on_road_debug(image)
                pathFinder.cur_image = image
                phiD = pathFinder.calc_phi_d()
                cv2.imshow("Rpi lane detection", pathFinder.project_on_road())
                key = cv2.waitKey(1) & 0xFF

                # clear the stream in preparation for the next frame
                rawCapture.truncate()
                rawCapture.seek(0)

                # if the `q` key was pressed, break from the loop
                if key == ord("q"):
                    break
            except:
                releaseAllLocks() #if there is an error release all the locks


########################################################################################################################
#END CAMERA THREAD CLASS
########################################################################################################################

########################################################################################################################
#START ARDUINO COM CLASS
########################################################################################################################
class ArduinoComThread(threading.Thread):
    def run(self):
        global curLat, curLon, phiD, desiredOffset, curOffset, camLookLeft, lastNode, gpsV
        global curLatLock, curLonLock, phiDLock, desiredOffsetLock, curOffsetLock, camLookLeftLock, lastNodeLock, gpsVLock, locks
        receiveFromArd = {'gpsLat': (curLat, curLatLock), 'gpsLon': (curLon, curLonLock), 'gpsV': (gpsV, gpsVLock), 'gpsPsi': (gpsPsi, gpsPsiLock)}
        sendToArd = {'phiD': (phiD, phiDLock), 'speedD': (speedD, speedDLock)}
        sendCtr = 0
        receiveCtr = 0
        while True:
            if sendCtr >= len(sendToArd):
                sendCtr = 0
            if receiveCtr >= len(receiveFromArd):
                receiveCtr = 0
            try:
                #send data
                sendDataName = sendToArd.keys()[sendCtr]
                varToSend, varToSendLock = sendToArd[sendDataName]
                varToSendLock.acquire()
                ArduinoCom.setData(sendDataName, varToSend)
                varToSendLock.release()
                ArduinoCom.sendData(sendDataName)

                #acquire data
                acqDataName = sendToArd.keys()[receiveCtr]
                varToAcq, varToAcqLock = sendToArd[acqDataName]
                ArduinoCom.getData(acqDataName)
                varToAcqLock.acquire()
                varToAcq = ArduinoCom.getData(acqDataName)
                varToAcqLock.release()


                #incr counter
                sendCtr += 1
                receiveCtr += 1

            except:
                releaseAllLocks()


########################################################################################################################
#END ARDUINO COM CLASS
########################################################################################################################

if __name__ == "__main__":
    camThread = CameraThread()
    mapThread = MapThread()
    comThread = ArduinoComThread()

    #prevents background program from runnin on exit
    camThread.setDaemon(True)
    mapThread.setDaemon(True)
    comThread.setDaemon(True)

    #start the threads
    camThread.start()
    mapThread.start()
    comThread.start()

    #keep main thread alive
    while True:
        pass






