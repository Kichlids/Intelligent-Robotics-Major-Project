import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

img = cv2.imread('testimg.png')

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# 102, 31, 31
lower_red = np.array([0,50,50])
upper_red = np.array([10,255,255])

mask = cv2.inRange(hsv, lower_red, upper_red)

cv2.imshow('mask', mask)


im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#print(len(contours))
for c in contours:
    moments = cv2.moments(mask)

    if moments["m00"] != 0:
        cX = int(moments["m10"] / moments["m00"])
        cY = int(moments["m01"] / moments["m00"])
    else:
        cX = 0
        cY = 0
    
    print(cX)
    print(cY)



while (True):
    cv2.waitKey(3)