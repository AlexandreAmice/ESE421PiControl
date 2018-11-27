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
    def __init__(self, image= None):
        self.img = image
        self.imgShape = image.shape
        self.redCenter = self.imgShape[1]/2


    def findRib(self):
        image = cv2.GaussianBlur(self.img, (11, 11), 11)
        ############################
        # HSV SHIFT
        #########################
        imageHSV = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        cv2.imwrite('hsv.jpg', imageHSV)
        h, s, v = cv2.split(imageHSV)
        cv2.imwrite('h.jpg', h)
        cv2.imwrite('s.jpg', s)
        cv2.imwrite('v.jpg', v)
        hlow = 0
        hhigh = 100
        slow = 0
        shigh = 100
        vlow = 150
        vhigh = 220
        threshH = 255 * np.ones_like(h) - cv2.inRange(h, hlow, hhigh)
        threshS = 255 * np.ones_like(h) - cv2.inRange(s, slow, shigh)
        threshV = cv2.inRange(v, vlow, vhigh)
        combined = (threshH) & (threshV) & threshS
        combined = cv2.medianBlur(combined, 5)
        cv2.imwrite('hThresh.jpg', threshH)
        cv2.imwrite('sThresh.jpg', threshS)
        cv2.imwrite('vThresh.jpg', threshV)
        sideBySide(image, combined, "HSV.jpg")

        ############################
        # LAB SHIFT
        #########################
        imageLAB = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)
        cv2.imwrite('lab.jpg', imageLAB)
        l, a, b = cv2.split(imageLAB)
        cv2.imwrite('l.jpg', l)
        cv2.imwrite('a.jpg', a)
        cv2.imwrite('b.jpg', b)
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
        cv2.imwrite('lThresh.jpg', threshL)
        cv2.imwrite('aThresh.jpg', threshA)
        cv2.imwrite('bThresh.jpg', threshB)
        sideBySide(image, combined2, "LAB.jpg")

        combined3 = combined & combined2
        try:
            avgX = int(np.median(np.nonzero(combined3)[1]))
            avgY = int(np.median(np.nonzero(combined3)[0]))
            boxW = 20
            boxH = 20

            cv2.rectangle(self.img, (avgX - boxW, avgY - boxH), (avgX + boxW, avgY + boxH), (0, 255, 0), 3)
            toShow = sideBySide(self.img, combined, 'total.jpg')
            self.redCenter = avgX
            return toShow
        except:
            return self.img

    def setImage(self,img):
        self.img = img

    def calcPsiOffset(self):
        return int((self.imgShape[1]-self.redCenter)/15.0)


if __name__ == '__main__':
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


        cv2.imshow("Rpi lane detection", ribbonFinder.findRib())
        key = cv2.waitKey(1) & 0xFF

        # clear the stream in preparation for the next frame
        rawCapture.truncate()
        rawCapture.seek(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

