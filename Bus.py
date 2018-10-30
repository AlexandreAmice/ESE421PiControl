import threading
import picamera
from picamera.array import PiRGBArray
import PathFinder
import warnings
import time
from MapObj import MapObj
import cv2

########################################################################################################################
#START GLOBAL DATA
########################################################################################################################
curLat = 39.9509507
curLon =  -75.185327
phiD = 0
desiredOffset = 3
curOffset = 3
camLookLeft = True

########################################################################################################################
#END GLOBAL DATA
########################################################################################################################


########################################################################################################################
#START MAP OBJECT THREAD CLASS
########################################################################################################################
class MapThread(threading.Thread):
    def run(self):
        global camLookLeft
        pathPlan = MapObj('mapPennParkNodes.txt', 'mapPennParkEdges.txt')
        while True:
            camLookLeft = pathPlan.getTurnLeft()
            if (curLat, curLon) != pathPlan.getGPSCoord():
                pathPlan.setGPSCoord(curLat, curLon)

            nearestPath,closestPoint, minDist = pathPlan.findNearestPath(curLat,curLon)
            if nearestPath[0] in pathPlan.curPlan and nearestPath[1] in pathPlan.curPlan:
                lastNode = pathPlan.lastNode
            if nearestPath != pathPlan.curRoad:
                pathPlan.guessRoadData()
                if pathPlan.lastNode:
                    pass
                pathPlan.planPath()






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






########################################################################################################################
#END CAMERA THREAD CLASS
########################################################################################################################



