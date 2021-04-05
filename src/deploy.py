#!/usr/bin/python3
import roslib
roslib.load_manifest('SAFMC-21-D2-Team2')
import sys
import rospy
import numpy as np
import cv2 as cv
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from down_cv import landing_pad, green_square
import time

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

# The current square it's looking for
curr_square = 1
# The current state, 0: leaving, 1: finding 2: approaching, 3: dropping
curr_state = 0

target_pub = rospy.Publisher('target', PositionTarget, queue_size=10)
land_pub = rospy.Publisher('/cmd_land', String, queue_size=10)
drop_pub = rospy.Publisher('/cmd_drop', String, queue_size=10)


drop_x=0
drop_y=0
prev_x=drop_x
prev_y=drop_y
def odom_callback(odometry):
    global drop_x, drop_y
    # print(odometry.pose.pose.position.x)
    drop_x=odometry.pose.pose.position.x
    drop_y=odometry.pose.pose.position.y
odom_sub = rospy.Subscriber('/camera/odom/sample', Odometry, odom_callback)
rospy.init_node('deploy_node', anonymous=True)
rate = rospy.Rate(20)

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


# Opening from video file, for dev/test
# cap = cv.VideoCapture('/home/kevinskwk/Videos/safmc/output4.avi')
rospy.sleep(1)
# Get the centre coordinate of the frame
ret, frame = cap.read()
centre = (frame.shape[1]/2, frame.shape[0]*2/3)
print(centre)
stable_count = 0
error = 13
p = 0.0013
# d = 0.003
vmax = 0.6

target_pub.publish(construct_target(0, 0, 1.0))
vx = 0
vy = 0
z = 0.6

while cap.isOpened() and not rospy.is_shutdown():
    ret, frame = cap.read()
    if curr_state < 4:
        res = green_square(frame, display=False)
    else:
        res = landing_pad(frame, display=False)
    # print(res)
    num = len(res)
    if num == 1:
        if curr_state == 1:  # Found
            curr_state = 2
    else:
        if curr_state == 0:
            stable_count += 1
            if stable_count > 8:
                stable_count = 0
                curr_state = 1
    
    if curr_state == 2 and ((curr_square < 4 and stable_count > 35) or \
       (curr_square == 4 and stable_count > 2)):  # stablized
        curr_state = 3
        prev_x=drop_x
        prev_y=drop_y
        stable_count = 0

    if curr_state <= 1:  # leaving or finding
        z = 0.6
        if curr_square <= 1:
            vx = 0.4
            vy = 0.4
        elif curr_square == 2:
            vx = 0.4
            vy = 0
        elif curr_square == 3:
            vx = 0
            vy = -0.4
        else:
            vx = 0.4
            vy = 0.4

    elif curr_state == 2 and num == 1: # approaching
        z = 1.0
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

    if curr_state == 3:
        if curr_square < 4:
            vx = 0.0 +(drop_x-prev_x)
            vy = 0.0 +(drop_y-prev_y+0.15)
            if z > 0.35:
                z -= 0.01
            else:
                print('dropping)')
                if stable_count==0:
                    drop_pub.publish(String("DROP"))
                stable_count += 1
            if stable_count > 21:
                curr_state = 0
                curr_square += 1
                stable_count = 0
        else:
            print("Landing")
            land_pub.publish(String("LAND"))
            break

    target_pub.publish(construct_target(vx, vy, z))
    print('square:', curr_square, 'state:', curr_state)
    print('vx:', vx, 'vy:', vy)
    rate.sleep()

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
