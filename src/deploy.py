#!/usr/bin/python3
import roslib
roslib.load_manifest('SAFMC-21-D2-Team2')
import sys
import rospy
import numpy as np
import cv2 as cv
from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget
from green_square import green_square

def construct_target(vx, vy, z, yaw_rate=0):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()

    target_raw_pose.coordinate_frame = 8

    target_raw_pose.velocity.x = vx
    target_raw_pose.velocity.y = vy
    target_raw_pose.position.z = z
    target_raw_pose.yaw_rate = yaw_rate

    target_raw_pose.type_mask = 1987

    return target_raw_pose

'''
def construct_vel_target(x, y, z):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()

    target_raw_pose.coordinate_frame = 8

    target_raw_pose.velocity.x = x
    target_raw_pose.velocity.y = y
    target_raw_pose.velocity.z = z

    target_raw_pose.type_mask = 3527

    return target_raw_pose
'''

# The current square it's looking for
curr_square = 0
# The current state, 0: leaving, 1: finding 2: approaching, 3: dropping
curr_state = 0

target_pub = rospy.Publisher('target', PositionTarget, queue_size=10)
land_pub = rospy.Publisher('/cmd_land', String, queue_size=10)
rospy.init_node('deploy_node', anonymous=True)
rate = rospy.Rate(10)

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
cap = cv.VideoCapture('/home/kevinskwk/Videos/safmc/output4.avi')
rospy.sleep(2)
# Get the centre coordinate of the frame
ret, frame = cap.read()
centre = (frame.shape[0]/2, frame.shape[1]/2)
stable_count = 0
error = 5
p = 0.003
vmax = 0.6

target_pub.publish(construct_target(0, 0, 1.0))
vx = 0
vy = 0
z = 1.0

while cap.isOpened() and not rospy.is_shutdown():
    ret, frame = cap.read()
    res = green_square(frame)
    # print(res)
    num = len(res)
    if num == 1:
        if curr_state == 1:  # Found
            curr_state = 2
    else:
        if curr_state == 0:
            curr_state = 1
            curr_square += 1
    
    if curr_state == 2 and stable_count > 50:  # stablized
        curr_state = 3
        stable_count = 0

    if curr_state <= 1:  # leaving or finding
        z = 1.0
        if curr_square == 0:
            vx = 0.1
            vy = 0.1
        elif curr_square == 1:
            vx = 0.1
            vy = 0
        elif curr_square == 2:
            vx = 0
            vy = -0.1
        else:
            vx = 0.1
            vy = 0.1
            stable_count += 1
            if stable_count > 30:
                print("Landing")
                land_pub.publish(String("LAND"))

    elif curr_state == 2 and num == 1: # approaching
        z = 1.0
        vx = centre[0] - res[0][0]
        vy = centre[1] - res[0][1]
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

    if curr_state == 3:
        vx = 0.0
        vy = 0.0
        if z > 0.3:
            z -= 0.01
        else:
            # TODO: send drop command
            stable_count += 1
        if stable_count > 30:
            curr_state = 0
            stable_count = 0

    target_pub.publish(construct_target(vx, vy, z))
    rate.sleep()

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
