import numpy as np
import math
import cv2
import time
import warnings
from scipy import stats
import random

#import picamera
#from picamera.array import PiRGBArray
warnings.filterwarnings('error')
def sideBySide(img1, img2, destName):
    dims1 = img1.shape
    dims2 = img2.shape
    if len(dims1) < len(dims2):
        temp = np.dstack((img1,img1,img1))
        toWrite = np.hstack((temp,img2))
    elif len(dims1) > len(dims2):
        temp = np.dstack((img2,img2,img2))
        toWrite = np.hstack((img1, temp))
    else:
        toWrite = np.hstack((img1, img2))
    cv2.imwrite(destName, toWrite)

def maxContour(contours):
    maxCont = None
    maxIdx = -1
    maxLen = -1 #cv2.arcLength(maxCont, False)
    for i, c in enumerate(contours):
        nextLen = cv2.arcLength(c, False)
        if nextLen > maxLen:
            maxCont = c
            maxIdx = i
            maxLen = nextLen
    return maxCont,maxIdx

class FindEdge():
    def __init__(self, img, removePixels = 0):
        # camera calibration parameters (if we get these)
        self.cam_mtx = None
        self.cam_dst = None

        # processed image for edge detection
        self.proc_img = None
        self.edges = None

        #current image
        self.x_dim = img.shape[1]
        self.y_dim = img.shape[0]
        self.carX, self.carY = self.x_dim / 2, self.y_dim
        self.removePixels = removePixels
        self.cropped_image = img[removePixels:, :]
        self.y_dimC = self.cropped_image.shape[0]
        self.x_dimC = self.cropped_image.shape[1]
        self.set_new_image(img)




        # windowing parameters for edge detection.
        # WARNING THESE ARE REALLY SENSITIVE TAKE CARE WHEN CHANGING
        self.window_height = 10
        self.window_width = 10
        self.margin = 20

        #what side of the road we want to look for
        self.look_left = False
        #number of frames since losing the road
        self.lostCount = 0




        #Left and Right Line
        self.rightLine = None
        self.leftLine = None


    def set_new_image(self, image):
        self.cur_image = image
        self.cropped_image = self.cur_image[self.removePixels:, :]
        self.y_dim, self.x_dim, _ = image.shape
        self.y_dimC, self.x_dimC, _ = self.cropped_image.shape
        self.set_preproc_cur_image()

    def set_line(self, vx, vy, x, y, left):
        fun = lambda xhat: int(((xhat - x) * vy / vx) + y)
        if left:
            self.leftLine = fun
        else:
            self.rightLine = fun



    def set_preproc_cur_image(self):
        '''
        process the cropped image in the hope of finding an edge
        :param self:
        :return:
        '''
        imageHSV = cv2.cvtColor(cv2.medianBlur(self.cropped_image, 3), cv2.COLOR_RGB2HSV)
        y_dim ,x_dim, _ = imageHSV.shape
        h, s, v = cv2.split(imageHSV)
        xBox = range(x_dim / 2 - 5, x_dim / 2 + 5)
        yBox = range(y_dim - 10, y_dim)

        hmean = np.mean(h[yBox, xBox])
        smean = np.mean(s[yBox, xBox])
        vmean = np.mean(v[yBox, xBox])

        epsH = 30
        epsS = 20
        epsV = 30
        threshH = cv2.inRange(h, hmean - epsH, hmean + epsH)
        threshS = cv2.inRange(s, smean - epsS, smean + epsS)
        threshV = cv2.inRange(v, vmean - epsV, vmean + epsV)
        combined = threshS | (threshH & threshV)#(threshH) & (threshS)
        combined = cv2.medianBlur(combined, 5)
        temp = combined
        combined = cv2.Canny(combined, 100, 200)
        self.proc_img = combined


        #for debugging
        cv2.imwrite('HSV.jpg', imageHSV)
        cv2.imwrite('hThresh.jpg', threshH)
        cv2.imwrite('sThresh.jpg', threshS)
        cv2.imwrite('vThresh.jpg', threshV)
        cont, contours, hierarchy = cv2.findContours(self.proc_img[:, :], cv2.RETR_TREE,
                                                     cv2.CHAIN_APPROX_SIMPLE)
        c, i = maxContour(contours)
        cv2.imwrite('combined.jpg', cv2.drawContours(temp,contours,i, (255,0,0), 3))



    def collect_edge_R(self):
        cropBy = self.x_dimC/2
        image = self.proc_img[:, -cropBy:]

        cont, contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        c, i = maxContour(contours)
        temp = np.copy(c)
        #offset contour back to full image
        temp[:,:,0] = temp[:,:,0] + self.x_dimC/2*np.ones_like(c[:,:,0])
        temp[:, :, 1] = temp[:, :, 1] + self.removePixels* np.ones_like(c[:, :, 0])
        thresh = 200 if not self.look_left else 500
        if c is not None and cv2.arcLength(c, False) > thresh:
            [vx, vy, x, y] = cv2.fitLine(temp, cv2.DIST_L2, 0, 0.01, 0.01)
            self.set_line(vx,vy,x,y,False)
            self.lostCount = 0 #reset counter for finding the road
            cv2.imwrite('collectR.jpg', cv2.drawContours(image, contours, i, (255, 0, 0), 3))
        else:
            #if we can't find the road
            if self.lostCount >= 10:
                self.rightLine = None
            cv2.imwrite('collectR.jpg', cv2.drawContours(image, contours, -1, (255, 0, 0), 3))
            self.lostCount += 1



    def collect_edge_L(self):
        cropBy = self.x_dimC / 3
        image = self.proc_img[:, :cropBy]
        cont, contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE,
                                                     cv2.CHAIN_APPROX_SIMPLE)
        c, i = maxContour(contours)
        temp = np.copy(c)
        temp[:, :, 1] = temp[:, :, 1] + self.removePixels * np.ones_like(c[:, :, 0])
        thresh = 200 if self.look_left else 500
        if c is not None and cv2.arcLength(c, False) > thresh:
            [vx, vy, x, y] = cv2.fitLine(temp, cv2.DIST_L2, 0, 0.01, 0.01)
            self.set_line(vx,vy,x,y,True)
            self.lostCount = 0  # reset counter for finding the road
            cv2.imwrite('collectL.jpg',
                        cv2.drawContours(image, contours, i, (255, 0, 0), 3))
        else:
            # if we can't find the road
            if self.lostCount >= 10:
                self.leftLine = None
            cv2.imwrite('collectL.jpg',
                        cv2.drawContours(image, contours, -1, (255, 0, 0), 3))
            self.lostCount += 0

    def draw_r_edge(self):
        if self.rightLine is not None:
            lefty = self.rightLine(self.x_dim/2)
            righty = self.rightLine(self.x_dim)
            temp = np.copy(self.cur_image)
            #cv2.rectangle(temp, (self.carX+35, self.carY), (self.carX,self.carY-243), (0, 255, 0), 4)
            cv2.line(temp, (self.x_dim, righty), (self.x_dim/2, lefty), (0, 0, 255), 5)
            return temp
        return self.cur_image

    def draw_l_edge(self):
        if self.leftLine is not None:
            lefty = self.leftLine(0)
            righty = self.leftLine(self.x_dim/2)
            temp = np.copy(self.cur_image)
            cv2.line(temp, (self.x_dim/2, righty), (0, lefty), (0, 255, 0), 5)
            return temp
        return self.cur_image

    def calc_road_offset(self):
        if self.look_left:
            self.collect_edge_L()
            curLine = self.leftLine
        else:
            self.collect_edge_R()
            curLine = self.rightLine

        if curLine is not None:
            #need to consider line in the form ax+by + c = 0. Curline is stored in form y=mx+b
            m = float(curLine(self.x_dim)-curLine(0))/self.x_dim
            a = m
            b = -1
            c = curLine(0)

            dist = abs(a*self.carX+b*self.carY+c)/math.sqrt(a**2+b**2)
            #print dist
            return dist
        return -1


if __name__ == "__main__":
    #image = cv2.imread('CameraTests/Test1.jpg')
    rand = random.randint(0,100)
    print rand
    image = cv2.imread('PennParkPics/Picture ' + str(rand)+'.jpg')
    temp = np.copy(image)
    y,x, _ = image.shape
    cv2.line(temp, (x/2,y),(x/2,0), (255,0,0),3)
    cv2.imwrite('image.jpg', temp)
    edgeFinder = FindEdge(image,y/2)
    edgeFinder.look_left = False
    edgeFinder.collect_edge_L()
    edgeFinder.collect_edge_R()
    edgeFinder.calc_road_offset()
    lines = cv2.addWeighted(edgeFinder.draw_r_edge(),0.5,edgeFinder.draw_l_edge(),0.5,0)
    cv2.imwrite('lines.jpg', lines)
    # cv2.imwrite('edgesR.jpg', edgeFinder.draw_r_edge())
    # cv2.imwrite('edgesL.jpg', edgeFinder.draw_l_edge())



