import numpy as np
import math
import cv2
import time
import warnings
from scipy import stats
import picamera
from picamera.array import PiRGBArray

def sideBySide(img1, img2, destName):
    dims1 = img1.shape
    dims2 = img2.shape
    if len(dims1) < len(dims2):
        temp = np.dstack((img1,img1,img1))
        toWrite = np.hstack((temp,img2))
    elif len(dims1) > len(dims2):
        temp = np.dstack((img2,img2,img2))
        toWrite = np.hstack((img1, temp))

    return toWrite
    #cv2.imwrite(destName, toWrite)

class findRibbon():
    def __init__(self, image = None):
        self.img = image
        if self.img is not None:
            self.imgShape = image.shape
            self.redCenter = self.imgShape[1]/2
        else:
            self.imgShape = None
            self.redCenter = None
            


    def findRib(self):
        #print 'start findRib'
        image = cv2.GaussianBlur(self.img, (11, 11), 11)
        ############################
        # HSV SHIFT
        #########################
        imageHSV = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        h, s, v = cv2.split(imageHSV)
        white = 255*np.ones_like(h)
        hlow = 0
        hhigh = 100
        slow = 0
        shigh = 100
        vlow = 150
        vhigh = 220
        threshH = 255 * white- cv2.inRange(h, hlow, hhigh)
        threshS = 255 * white - cv2.inRange(s, slow, shigh)
        threshV = cv2.inRange(v, vlow, vhigh)
        combined = (threshH) & (threshV) & threshS
        combined = cv2.medianBlur(combined, 5)
        #print 'end HSV'
        
        ############################
        # LAB SHIFT
        #########################
        imageLAB = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)
        l, a, b = cv2.split(imageLAB)
        llow = 50
        lhigh = 150
        alow = 150
        ahigh = 255
        blow = 50
        bhigh = 100
        threshL = cv2.inRange(l, llow, lhigh)
        threshA = cv2.inRange(a, alow, ahigh)
        threshB = cv2.inRange(b, blow, bhigh)
        combined2 = (threshL) & (threshA) & threshB
        combined3 = combined & combined2
        #print 'end LAB'
        try:
            avgX = int(np.median(np.nonzero(combined3)[1]))
            avgY = int(np.median(np.nonzero(combined3)[0]))
            boxW = 20
            boxH = 20
            
            cv2.rectangle(self.img, (avgX - boxW, avgY - boxH), (avgX + boxW, avgY + boxH), (0, 255, 0), 3)
            toShow = sideBySide(self.img, combined, 'total.jpg')
            self.redCenter = avgX
            #print 'over set'
            cv2.line(toShow, (self.imgShape[1]/2,self.imgShape[0]), (self.imgShape[1]/2, 0), (0,255,0))
            return toShow
        except:
            #print 'over failed'
            return self.img

    def setImage(self,img):
        self.img = img
        self.imgShape = img.shape
        self.redCenter = self.imgShape[1]/2

    def calcPsiOffset(self):
        return 5*math.atan2((self.imgShape[1]/2-self.redCenter),self.imgShape[0])


if __name__ == "__main__":
    
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
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        ribbonFinder.setImage(image)
        ribbonFinder.findRib()


        #cv2.imshow("Find Ribbon", ribbonFinder.findRib())
        key = cv2.waitKey(1) & 0xFF

        # clear the stream in preparation for the next frame
        rawCapture.truncate()
        rawCapture.seek(0)

        print ribbonFinder.calcPsiOffset()
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

