#!/usr/bin/python3
import roslib
roslib.load_manifest('SAFMC-21-D2-Team2')
import sys
import rospy
import numpy as np
import cv2 as cv

''' Using uvc cam
nocam=True
for i in range(6,10):
    cap = cv.VideoCapture(i)
    ret, frame = cap.read()
    time.sleep(2)
    if not cap.isOpened():
        print("Cannot open camera ",i)
        #exit()
    else:
        nocam=False
        break
if nocam:
    exit()
'''

# Opening from video file, for dev/test
cap = cv.VideoCapture('/home/kevinskwk/Videos/safmc/output2.avi')

UpperH = 61         # 0-180
UpperS = 177        # 0-255
UpperV = 173        # 0-255
LowerH = 37         # 0-180
LowerS = 67         # 0-255
LowerV = 62         # 0-255
MinSize = 2000      # Minimun number of white pixels of the mask
Filter = 7          # positive int

detect = False
cX, cY = 0, 0
rX, rY = 0, 0

while(cap.isOpened()):
    ret, frame = cap.read()
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lower_colour = np.array([LowerH, LowerS, LowerV])
    upper_colour = np.array([UpperH, UpperS, UpperV])
    mask = cv.inRange(hsv, lower_colour, upper_colour)
    mask = cv.medianBlur(mask, Filter*2+1)

    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    detect = False
    if len(contours) > 0:
        contours.sort(key=cv.contourArea, reverse=True)
        for c in contours:
            if cv.contourArea(c) > MinSize:
                detect = True
                M = cv.moments(c)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv.drawContours(frame, [c], -1, (255, 100, 255), 2)
                    x, y, w, h = cv.boundingRect(c)
                    cv.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                    rX = int(x+(w/2))
                    rY = int(y+(h/2))
                    cv.circle(frame, (rX, rY), 5, (0,255,255), -1)
                    cv.putText(frame, "centre", (rX - 25, rY - 25), 
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    cv.imshow('Mask', mask)
    cv.imshow("Image", frame)

    if detect:
        print('cX, cY:', cX, cY)
        print('rX, rY:', rX, rY)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()