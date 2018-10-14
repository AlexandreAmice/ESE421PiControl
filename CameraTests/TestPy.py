import cv2
import numpy as np

mask = cv2.imread('mask.jpg')
trans = cv2.imread('trans.jpg')
edgesMask = cv2.Canny(mask,20,80)
edgesTrans = cv2.Canny(trans,20,80)
cv2.imwrite('edgesMask.jpg', edgesMask)
cv2.imwrite('edgesTrans.jpg', edgesTrans)




