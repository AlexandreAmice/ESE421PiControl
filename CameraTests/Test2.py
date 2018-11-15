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

    cv2.imshow(destName, toWrite)


img = cv2.imread('TestRib1.jpg')
image = cv2.GaussianBlur(img, (11,11), 11)
ylen,xlen,d = image.shape
trimTop =ylen/2
trimRight = xlen
lines = []


############################
#HSV SHIFT
#########################
imageHSV = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
cv2.imwrite('hsv.jpg',imageHSV)
h,s,v = cv2.split(imageHSV)
cv2.imwrite('h.jpg',h)
cv2.imwrite('s.jpg',s)
cv2.imwrite('v.jpg',v)
hlow = 0
hhigh = 100
slow = 0
shigh = 100
vlow = 150
vhigh = 220
threshH = 255*np.ones_like(h)-cv2.inRange(h,hlow, hhigh)
threshS = 255*np.ones_like(h)-cv2.inRange(s,slow, shigh)
threshV = cv2.inRange(v,vlow, vhigh)
combined = (threshH) & (threshV) & threshS
combined = cv2.medianBlur(combined,5)
cv2.imwrite('hThresh.jpg',threshH)
cv2.imwrite('sThresh.jpg',threshS)
cv2.imwrite('vThresh.jpg',threshV)
sideBySide(image, combined, "HSV.jpg")

############################
#LAB SHIFT
#########################
imageLAB = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)
cv2.imwrite('lab.jpg',imageLAB)
l,a,b = cv2.split(imageLAB)
cv2.imwrite('l.jpg',l)
cv2.imwrite('a.jpg',a)
cv2.imwrite('b.jpg',b)
llow = 50
lhigh = 150
alow = 150
ahigh = 255
blow = 50
bhigh = 100
threshL = cv2.inRange(l,llow, lhigh)
threshA = cv2.inRange(a,alow, ahigh)
threshB = cv2.inRange(b,blow, bhigh)
combined2 = (threshL) & (threshA) & threshB
cv2.imwrite('lThresh.jpg',threshL)
cv2.imwrite('aThresh.jpg',threshA)
cv2.imwrite('bThresh.jpg',threshB)
sideBySide(image, combined2, "LAB.jpg")

combined3 = combined & combined2
avgX = int(np.median(np.nonzero(combined3)[1]))
avgY = int(np.median(np.nonzero(combined3)[0]))
print avgX
print avgY
boxW = 20
boxH = 20

cv2.rectangle(img, (avgX-boxW, avgY-boxH), (avgX+boxW, avgY+boxH), (0, 255, 0), 3)
sideBySide(img, combined, 'total.jpg')






