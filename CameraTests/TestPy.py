import numpy as np
import cv2
import time
import warnings

image = cv2.imread('blackRoad.jpg')
img = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

low_black = np.array([50, 50, 50]) #if value is
high_black = np.array([100, 255, 255])
mask = cv2.inRange(img, low_black, high_black)
cv2.imwrite('mask.jpg', mask)
blurMask = cv2.GaussianBlur(mask, (7,7), 3)
cv2.imwrite('blurMask.jpg', blurMask)
cropped = mask[300:,500:]
cv2.imwrite('maskCrop.jpg', cropped)
cv2.imwrite('cannyCrop.jpg', cv2.Canny(cropped, 80,20))
#maskRGB = cv2.cvtColor(mask, cv2.COLOR_HSV2RGB)
#cv2.imwrite('maskRGB.jpg', maskRGB)

