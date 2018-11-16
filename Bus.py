import threading
import picamera
from picamera.array import PiRGBArray
import PathFinder
import warnings
import time
from MapObj import MapObj
import cv2
from threading import Lock
from ArduinoCom import ArduinoCom
from Follower import findRibbon

print __name__
########################################################################################################################
#START GLOBAL DATA
########################################################################################################################
curLat = 39.9509507
curLon = -75.185327
gpsV = 0
psiD = -20
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
    def run(self):
        print "Launching Cam Thread"
        global camLookLeft
        global psiD

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
                print "I'm here"
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                pathFinder.setLookLeft(camLookLeft)
                image = frame.array

                # show the frame
                # lines.project_on_road_debug(image)
                pathFinder.cur_image = image
                psiD = pathFinder.calc_phi_d()
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
#END ROADEDGE THREAD CLASS
########################################################################################################################

########################################################################################################################
#START ARDUINO COM CLASS
########################################################################################################################
class ArduinoComThread(threading.Thread):
    def run(self):
        print "Launching Com Thread"
        global curLat, curLon, psiD, desiredOffset, curOffset, camLookLeft, lastNode, gpsV
        global curLatLock, curLonLock, psiDLock, desiredOffsetLock, curOffsetLock, camLookLeftLock, lastNodeLock, gpsVLock, locks
        sendCtr = 0
        receiveCtr = 0
        com = ArduinoCom()
        while True:
            time.sleep(1)
            #receiveFromArd = {'gpsLat': (curLat, curLatLock), 'gpsLon': (curLon, curLonLock), 'gpsV': (gpsV, gpsVLock), 'gpsPsi': (gpsPsi, gpsPsiLock)}
            sendToArd = {'psiD': (psiD, psiDLock)} # 'speedD': (speedD, speedDLock)}

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

    ribThread = RibbonTrackThread()
    ribThread.setDaemon(True)
    ribThread.start()

    #mapThread = MapThread()
    #mapThread.setDaemon(True)
    #mapThread.start()

    #camThread = CameraThread()
    #camThread.setDaemon(True)
    #camThread.start()
    
    #keep main thread alive
    count = 0
    while True:
        temp = raw_input("new speedD")
##        psiDLock.acquire()
##        psiD = temp
##        psiDLock.release()
        
        speedDLock.acquire()
        speedD = temp
        speedDLock.release()
            
        
        






