#!/usr/bin/python3
# import roslib
# roslib.load_manifest('SAFMC-21-D2-Team2')
import sys
# import rospy
import numpy as np
import cv2 as cv
import time
import socket

# Using uvc cam
nocam=True

if(socket.gethostname()=="pootis-XPS-15-9570"):
    cam_range=range(2,6)
else:
    cam_range=range(6,10)
for i in cam_range:
    cap = cv.VideoCapture(i)
    cap.set(cv.CAP_PROP_FRAME_WIDTH , 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT , 480)
    ret, frame = cap.read()
    time.sleep(2)
    if not cap.isOpened():
        print("Cannot open camera ",i)
        #exit()
    else:
        while frame is None:
            ret, frame = cap.read()
            time.sleep(1)
        print(frame)
        nocam=False
        break
    
if nocam:
    exit()


# Opening from video file, for dev/test
# cap = cv.VideoCapture('/home/kevinskwk/Videos/safmc/output4.avi')

time.sleep(0.1)

cv.namedWindow('Mask')
cv.namedWindow('Frame')

def f(x):
    pass

cv.createTrackbar('UpperH', 'Mask', 0, 180, f)
cv.createTrackbar('UpperS', 'Mask', 0, 255, f)
cv.createTrackbar('UpperV', 'Mask', 0, 255, f)
cv.createTrackbar('LowerH', 'Mask', 0, 180, f)
cv.createTrackbar('LowerS', 'Mask', 0, 255, f)
cv.createTrackbar('LowerV', 'Mask', 0, 255, f)
cv.createTrackbar('MinSize','Frame', 0 , 2000, f)
cv.createTrackbar('Filter','Mask', 0 , 10, f)

#landing pad
# cv.setTrackbarPos('UpperH', 'Mask', 91)
# cv.setTrackbarPos('UpperS', 'Mask', 243)
# cv.setTrackbarPos('UpperV', 'Mask', 131)
# cv.setTrackbarPos('LowerH', 'Mask', 67)
# cv.setTrackbarPos('LowerS', 'Mask', 148)
# cv.setTrackbarPos('LowerV', 'Mask', 62)
# cv.setTrackbarPos('MinSize','Frame', 1000)
# cv.setTrackbarPos('Filter', 'Mask', 7)

#lED square

cv.setTrackbarPos('UpperH', 'Mask', 107)
cv.setTrackbarPos('UpperS', 'Mask', 213)
cv.setTrackbarPos('UpperV', 'Mask', 255)
cv.setTrackbarPos('LowerH', 'Mask', 64)
cv.setTrackbarPos('LowerS', 'Mask', 22)
cv.setTrackbarPos('LowerV', 'Mask', 183)
cv.setTrackbarPos('MinSize','Frame', 1000)
cv.setTrackbarPos('Filter', 'Mask', 7)

UpperH, UpperS, UpperV, LowerH, LowerS, LowerV, MinSize, Filter = 0, 0, 0, 0, 0, 0, 0, 0

while(cap.isOpened()):
    ret, frame = cap.read()

    UpperH = cv.getTrackbarPos('UpperH', 'Mask')
    UpperS = cv.getTrackbarPos('UpperS', 'Mask')
    UpperV = cv.getTrackbarPos('UpperV', 'Mask')
    LowerH = cv.getTrackbarPos('LowerH', 'Mask')
    LowerS = cv.getTrackbarPos('LowerS', 'Mask')
    LowerV = cv.getTrackbarPos('LowerV', 'Mask')
    MinSize = cv.getTrackbarPos('MinSize', 'Frame')
    Filter = cv.getTrackbarPos('Filter', 'Mask')

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lower_colour = np.array([LowerH, LowerS, LowerV])
    upper_colour = np.array([UpperH, UpperS, UpperV])
    mask = cv.inRange(hsv, lower_colour, upper_colour)
    mask = cv.medianBlur(mask, Filter*2+1)

    contours, hierarchy = cv.findContours(mask,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)

    count = 0
    for c in contours:
        if cv.contourArea(c) > MinSize:
            cv.drawContours(frame, [c], -1, (255, 100, 255), 2)
            count += 1
    print('Count:', count)

    cv.imshow('Mask', mask)
    cv.imshow("Frame", frame)
    cv.waitKey(1)
    # if cv.waitKey(1) & 0xFF == ord('q'):
    #     break

# When everything done, release the capture
print('UpperH:', UpperH)
print('UpperS:', UpperS)
print('UpperV:', UpperV)
print('LowerH:', LowerH)
print('LowerS:', LowerS)
print('LowerV:', LowerV)
print('MinSize:', MinSize)
print('Filter: ', Filter)

cap.release()
cv.destroyAllWindows()