#!/usr/bin/env python3

import rospy
import math
from mavros_msgs.msg import PositionTarget
from hole_detector import Hole_detector
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import cv2 as cv
from down_cv import landing_pad, green_square
from mavros_msgs.msg import PositionTarget
# from green_square import green_square
import time

#odometry
current_x=0
current_y=0
current_yaw=0
prev_x=current_x
prev_y=current_y
prev_yaw=current_yaw
def odom_callback(odometry):
    global current_x, current_y
    # print(odometry.pose.pose.position.x)
    current_x=odometry.pose.pose.position.x
    current_y=odometry.pose.pose.position.y
    #TODO: get yaw
odom_sub = rospy.Subscriber('/camera/odom/sample', Odometry, odom_callback)


def construct_target(vx, vy, z, yaw_rate):
    #mask = 4035 # xy vel + z pos
    #mask = 3011 # xy vel + z pos + yaw
    #mask = 4088 # xyz pos
    #mask = 3064 # xyz pos + yaw
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()

    target_raw_pose.coordinate_frame = 8

    target_raw_pose.velocity.x = vx
    target_raw_pose.velocity.y = vy
    target_raw_pose.position.z = z
    #target_raw_pose.velocity.z = z
    
    target_raw_pose.yaw_rate = yaw_rate

    target_raw_pose.type_mask = 1987
    #target_raw_pose.type_mask = mask

    return target_raw_pose

odom=Odometry()
x=0
y=0
z=0
distance=0

def odometry_callback(thing):
    global odom,x,y,z, distance
    odom=thing
    x=odom.pose.pose.position.x
    y=odom.pose.pose.position.y
    z=odom.pose.pose.position.z
    distance=(x**2+y**2)**0.5
    #print(odom.pose.pose.position.x)
    return

pos_sub = rospy.Subscriber(
            'camera/odom/sample', Odometry,
            odometry_callback)



land_pub = rospy.Publisher('/cmd_land', String, queue_size=10)
target_pub = rospy.Publisher('target', PositionTarget, queue_size=10)
#target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
rospy.init_node('cmd_node', anonymous=True)

hole_detector = Hole_detector(output=True)
rospy.sleep(1)
rate = rospy.Rate(10)

circlefound=False
circlecount=0
land=False
vx=0
vy=0
yr=0
z=1.1
while not rospy.is_shutdown():

    vx=0
    vy=0
    yr=0
    z=1.1
    hole_center=hole_detector.get_hough()
    space_center=hole_detector.get_space()
    if circlefound==False and hole_center is not None:
        print(hole_center)
        yr=-hole_center/7

        if -0.1<hole_center<0.1:
            circlecount+=1
            if circlecount>45:
                circlefound=True
                prev_x=current_x
                prev_y=current_y

    # elif land:
    #     print("landing")
    #     #TOTO: change to find green square  
    #     land_pub.publish(String("LAND"))
    elif circlefound:
        print("moving forward")
        vy=current_y-prev_y
        vx=0.2
        yr=0
    else:
        yr=0.1
    if distance >3.5:
        hole_detector.unsuscribe()
        break
        # land=True
    target_pub.publish(construct_target(vx, vy, z, yr))
    rate.sleep()

# stop the drone first
target_pub.publish(construct_target(0, 0, 1.3, 0))

# boxCV job done, delete to release resources
del hole_detector

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
centre = (frame.shape[1]/2, frame.shape[0]*2/3)
error = 13
p = 0.0013
vmax = 0.6
stable_count=0
while cap.isOpened() and not rospy.is_shutdown():
    ret, frame = cap.read()
    res = landing_pad(frame, display=False)
    num = len(res)
    if(num==1):
        #found
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
        if stable_count > 15:  # stablized
            print("Landing")
            land_pub.publish(String("LAND"))
            break
        print('vx:', vx, 'vy:', vy)
    else:
        yr = 0
        vy = 0
        vx = 0.1


    target_pub.publish(construct_target(vx, vy, 1.3, yr))
    
    rate.sleep()

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
