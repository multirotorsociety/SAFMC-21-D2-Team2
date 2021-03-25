#!/usr/bin/python
import roslib
roslib.load_manifest('SAFMC-21-D2-Team2')
import sys
import rospy
import numpy as np
import cv2 as cv
import time
import datetime 
today = datetime.datetime.now()
date_time = today.strftime("%m%d%Y%H%M%S")

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

fourcc = cv.VideoWriter_fourcc(*'XVID')
width  = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))   # float `width`
height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))  # float `height`
out = cv.VideoWriter('/home/mrsteam2/output'+date_time+'.avi', fourcc, 20.0, (width,  height))

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    #gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # Display the resulting frame
    #cv.imshow('frame', frame)
    out.write(frame)
    cv.waitKey(1)
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()