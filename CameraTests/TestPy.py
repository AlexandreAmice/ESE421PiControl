import cv2
import numpy as np

img = cv2.imread('blackRoad.jpg')
img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
low_black = np.array([50,50,50])
high_black = np.array([100,255,255])
mask = cv2.inRange(img, low_black, high_black)
final = cv2.bitwise_and(img, img, mask)

#imgBlur5 = cv2.GaussianBlur(img,(5,5),2)
#imgBlur3 = cv2.GaussianBlur(img,(3,3),2)
#cv2.imwrite('GaussianBlur5.jpg', imgBlur5)
#cv2.imwrite('GaussianBlur3.jpg', imgBlur3)
cv2.imwrite('HSV.jpg', img)
cv2.imwrite('mask.jpg', mask)
cv2.imwrite('final.jpg', final)

