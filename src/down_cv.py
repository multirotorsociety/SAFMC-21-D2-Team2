#!/usr/bin/python3
import sys
import numpy as np
import cv2 as cv
import time

def landing_pad(frame, display=False):
    UpperH = 91         # 0-180
    UpperS = 243        # 0-255
    UpperV = 131       # 0-255
    LowerH = 67       # 0-180
    LowerS = 148         # 0-255
    LowerV = 62         # 0-255
    MinSize = 1000      # Minimun number of white pixels of the mask
    Filter = 7          # positive int

    cX, cY = 0, 0
    rX, rY = 0, 0
    res = []

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lower_colour = np.array([LowerH, LowerS, LowerV])
    upper_colour = np.array([UpperH, UpperS, UpperV])
    mask = cv.inRange(hsv, lower_colour, upper_colour)
    mask = cv.medianBlur(mask, Filter*2+1)

    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours.sort(key=cv.contourArea, reverse=True)
    for c in contours:
        if cv.contourArea(c) > MinSize:
            M = cv.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                x, y, w, h = cv.boundingRect(c)
                rX = int(x+(w/2))
                rY = int(y+(h/2))
                res.append([rX, rY, w*h])
                if display:
                    cv.drawContours(frame, [c], -1, (255, 100, 255), 2)
                    cv.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                    cv.circle(frame, (rX, rY), 5, (0,255,255), -1)
                    cv.circle(frame, (320, 240), 4, (0,0,255), -1)
                    cv.putText(frame, "centroid", (rX - 25, rY - 25), 
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    print('rX, rY:', rX, rY)
                if w*h > 80000:
                    break
    if display:
        cv.imshow('Mask', mask)
        cv.imshow("Image", frame)
        cv.waitKey(1)

    return res

def green_square(frame, display=False):
    UpperH = 107        # 0-180
    UpperS = 213        # 0-255
    UpperV = 255        # 0-255
    LowerH = 64        # 0-180
    LowerS = 0         # 0-255
    LowerV = 183         # 0-255
    MinSize = 1000      # Minimun number of white pixels of the mask
    Filter = 7          # positive int

    cX, cY = 0, 0
    rX, rY = 0, 0
    res = []

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lower_colour = np.array([LowerH, LowerS, LowerV])
    upper_colour = np.array([UpperH, UpperS, UpperV])
    mask = cv.inRange(hsv, lower_colour, upper_colour)
    mask = cv.medianBlur(mask, Filter*2+1)

    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours.sort(key=cv.contourArea, reverse=True)
    for c in contours:
        if cv.contourArea(c) > MinSize:
            M = cv.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                x, y, w, h = cv.boundingRect(c)
                rX = int(x+(w/2))
                rY = int(y+(h/2))
                res.append([rX, rY, w*h])
                if display:
                    cv.drawContours(frame, [c], -1, (255, 100, 255), 2)
                    cv.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                    cv.circle(frame, (rX, rY), 5, (0,255,255), -1)
                    cv.circle(frame, (320, 240), 4, (0,0,255), -1)
                    cv.putText(frame, "centroid", (rX - 25, rY - 25), 
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    print('rX, rY:', rX, rY)
                if w*h > 80000:
                    break
    if display:
        cv.imshow('Mask', mask)
        cv.imshow("Image", frame)
        cv.waitKey(1)

    return res

def main(args):
    # Using uvc cam
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

    # Opening from video file, for dev/test
    #cap = cv.VideoCapture('/home/kevinskwk/Videos/safmc/output4.avi')

    while(cap.isOpened()):
        ret, frame = cap.read()
        res = green_square(frame, display=True)
        print(res)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
