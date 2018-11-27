import numpy as np
import cv2
import time
import warnings
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


image = cv2.imread('Test3.jpg')
ylen,xlen,d = image.shape
trimTop =ylen/2
trimRight = xlen
lines = []

#while len(lines) != 1 and (trimTop < ylen and trimRight > xlen/2):
# temp = image
# r,g,b = cv2.split(temp)
# subtract = r-g-b
# cv2.imwrite('r.jpg',r)
# cv2.imwrite('g.jpg',g)
# cv2.imwrite('b.jpg',g)
# cv2.imwrite('subtract.jpg', subtract)
# thresh = cv2.inRange(subtract, 0, 80)
# cv2.imwrite('whitewash.jpg',thresh)
# #thresh = cv2.merge((r,thresh,b))
# #thresh = cv2.medianBlur(thresh, 21)
# cont, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# h,s,v = cv2.split(temp)
# #cv2.drawContours(temp, contours, -1, (255,0,0), 3)
# canny = cv2.Canny(thresh, 100,200)
# minLineLength = 70
# maxLineGap = 60
# lines = cv2.HoughLinesP(canny,rho=1,theta=np.pi/180,threshold=10,minLineLength=minLineLength,maxLineGap=maxLineGap)
# cont, contours, hierarchy = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# cv2.drawContours(temp, contours, 4, (255,0,0), 3)
# #print len(contours)
# cv2.imwrite('tresh.jpg', thresh)
# cv2.imwrite('canny.jpg',canny)
#

############################
#Attempt via thresholding on center pixels
###################################
imageHSV = cv2.cvtColor(cv2.medianBlur(image,3), cv2.COLOR_RGB2HSV)
h,s,v = cv2.split(imageHSV)
xBox = range(xlen/2-5,xlen/2+5)
yBox = range(ylen-10, ylen)
hmean = np.mean(h[yBox,xBox])
smean = np.mean(s[yBox,xBox])
vmean = np.mean(v[yBox,xBox])

epsH = 20
epsS = 20
epsV = 50
threshH = cv2.inRange(h,hmean-epsH, hmean+epsH)
threshS = cv2.inRange(s,smean-epsS, smean+epsS)
threshV = cv2.inRange(v,vmean-epsV, vmean+epsV)
combined = (threshH) & (threshS)
combined = cv2.medianBlur(combined,5)
combined = cv2.Canny(combined,100,200)
cv2.imwrite('hThresh.jpg',threshH)
cv2.imwrite('sThresh.jpg',threshS)
cv2.imwrite('vThresh.jpg',threshV)
cv2.imwrite('combined.jpg',combined)
cont, contours, hierarchy = cv2.findContours(combined[:,xlen/2:], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
c, i = maxContour(contours)
temp = np.zeros_like(image)
row,cols = combined.shape[:2]
[vx,vy,x,y] = cv2.fitLine(c, cv2.DIST_L2,0,0.01,0.01)
lefty = int((-x*vy/vx) + y)
righty = int(((cols-x)*vy/vx)+y)
cv2.line(image,(cols-1+xlen/2,righty),(0+xlen/2,lefty),(0,255,0),2)
#cv2.drawContours(temp, contours, -1, (255,0,0), 3)


cv2.drawContours(temp, contours, i, (255,0,0), 3)
cv2.imwrite('combinedCont.jpg',temp)

# print len(contours[319])
# longest = contours.index(max(contours, key=len))
# print len(contours)
# temp = image
# cv2.drawContours(temp, longest, -1, (255,0,0), 3)
# cv2.imwrite('temp.jpg', temp)
temp2 = np.zeros_like(image)
cv2.drawContours(temp2, contours, i, (255,0,0), 3)
sideBySide(image, temp, "HSV.jpg")










# low_black = np.array([50, 50, 50]) #if value is
# high_black = np.array([100, 255, 255])
# mask = cv2.inRange(img, low_black, high_black)
# cv2.imwrite('mask.jpg', mask)
# blurMask = cv2.GaussianBlur(mask, (7,7), 3)
# cv2.imwrite('blurMask.jpg', blurMask)
# cropped = mask[300:,500:]
# cv2.imwrite('maskCrop.jpg', cropped)
# cv2.imwrite('cannyCrop.jpg', cv2.Canny(cropped, 80,20))
# #maskRGB = cv2.cvtColor(mask, cv2.COLOR_HSV2RGB)
# #cv2.imwrite('maskRGB.jpg', maskRGB)

