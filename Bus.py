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

########################################################################################################################
# START SOPHIA CODE
########################################################################################################################


phiD = 0
desiredOffset = 3
curOffset = 3
camLookLeft = True

    #get psi_map and speed from main and send to path planner
    speedLimit = pathPlan.getRoadSpeedCurGPS() #use lat and lon, determine max speed on this path

    #send GPS to path planner from I2C
    curLat, curLon = mainI2C.getGPS()

    #get psi_r from camera
    psiR = camera.getPsiR()

    #look left / right (included above)

    #get psi_d, speed from Arduino
    psiD = mainI2C.getPsiD()
    speedCurrent = mainI2C.getSpeed() ##not sure how sending speed --> need to decide as lab group whether using PWM

    #read in from text file
    #assumes data organized as psiD then speedCurrent
    with open("data.txt") as in_file:
        # create a csv reader object
        csv_reader = reader(in_file)

        # extract headers
        headers = [x.strip() for x in next(csv_reader)]

        # go over each line 
        for line in csv_reader:
            # if line is not empty
            if line:
                psiD = line
                speedCurrent = line
    
    #send GPS using I2C
    mainI2C.setGPS(lat, lon)

########################################################################################################################
# END SOPHIA CODE
# between Pi and Arduino
# i2C
########################################################################################################################
