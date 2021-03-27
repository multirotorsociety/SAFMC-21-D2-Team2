#!/usr/bin/env python3
import roslib
roslib.load_manifest('SAFMC-21-D2-Team2')
import rospy
import sys
import math
from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget
from box import BoxCV
import numpy as np
import cv2 as cv
from green_square import green_square
import time

def construct_target(vx, vy, z, yaw_rate):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()

    target_raw_pose.coordinate_frame = 8

    target_raw_pose.velocity.x = vx
    target_raw_pose.velocity.y = vy
    target_raw_pose.position.z = z
    target_raw_pose.yaw_rate = yaw_rate

    target_raw_pose.type_mask = 1987

    return target_raw_pose

target_pub = rospy.Publisher('target', PositionTarget, queue_size=10)
land_pub = rospy.Publisher('/cmd_land', String, queue_size=10)
rospy.init_node('cmd_node', anonymous=True)

px = 0.25
pyaw = 0.25

total_yaw = 0
# state 0: idle, 1: navigating, 2: turning, 3: finding, 4: approaching
state = 0
stable_count = 0

box = BoxCV(output=True)
rate = rospy.Rate(20)
rospy.sleep(3)
# TODO: subscribe to odom and get initial position for reference
state = 1

vx = 0
vy = 0
yr = 0

while not rospy.is_shutdown():
    if state == 1 and total_yaw > 3*math.pi:  # finished navigating
        stable_count += 1
        if stable_count > 200:
            state = 2
            stable_count = 0
            total_yaw = 0
            break
    if state == 1:
        vy = -0.2
        theta = box.get_theta()
        vx = box.get_mean() - 1.0
        if abs(vx) <= 0.05:
            vx = 0.0
        else:
            vx *= px
            if vx > 0:
                vx = min(vx, 0.6)
            elif vx < 0:
                vx = max(vx, -0.6)

        yr = theta[1]
        if abs(yr) <= 0.01:
            yr = 0.0
        else:
            yr *= pyaw
            if yr > 0:
                yr = min(yr, 0.4)
            elif yr < 0:
                yr = max(yr, -0.4)
        
        target_pub.publish(construct_target(vx, vy, 1.1, yr))
        total_yaw += yr / 20.0

    rate.sleep()

# stop the drone first
target_pub.publish(construct_target(0, 0, 1.1, 0))

# boxCV job done, delete to release resources
del box

# Using uvc cam
nocam = True
for i in range(6,10):
    cap = cv.VideoCapture(i)
    ret, frame = cap.read()
    time.sleep(1)
    if not cap.isOpened():
        print("Cannot open camera ",i)
        #exit()
    else:
        nocam = False
        break
if nocam:
    exit()

ret, frame = cap.read()
centre = (frame.shape[1]/2, frame.shape[0]/2)
error = 10
p = 0.0015
vmax = 0.6

while cap.isOpened() and not rospy.is_shutdown():
    ret, frame = cap.read()
    res = green_square(frame, display=False)
    num = len(res)
    if num == 1 and state == 3:  # Found
        state = 4
    
    if state == 2:  # turning
        vx = 0
        vy = 0
        yr = 0.2
        if total_yaw > math.pi:
            state = 3
            total_yaw = 0

    if state == 3:
        vx = 0.2
        vy = 0
        yr = 0

    if state == 4 and num >= 1:
        yr = 0
        vy = centre[0] - res[0][0]
        vx = centre[1] - res[0][1]
        if abs(vx) < error:
            vx = 0.0
        else:
            vx *= p
            if vx > 0:
                vx = min(vx, vmax)
            elif vx < 0:
                vx = max(vx, -vmax)
        if abs(vy) < error:
            vy = 0.0
        else:
            vy *= p
            if vy > 0:
                vy = min(vy, vmax)
            elif vy < 0:
                vy = max(vy, -vmax)

        if vx == 0.0 and vy == 0.0:
            stable_count += 1
        if stable_count > 50:  # stablized
            print("Landing")
            land_pub.publish(String("LAND"))
            break
        print('vx:', vx, 'vy:', vy)

    target_pub.publish(construct_target(vx, vy, 1.1, yr))
    total_yaw += (yr / 20.0)
    print('state:', state)
    rate.sleep()

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
