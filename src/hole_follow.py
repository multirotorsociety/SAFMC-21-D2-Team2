#!/usr/bin/env python3

import rospy
import math
from mavros_msgs.msg import PositionTarget
from hole_detector import Hole_detector

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

target_pub = rospy.Publisher('target', PositionTarget, queue_size=10)
#target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
rospy.init_node('cmd_node', anonymous=True)

hole_detector = Hole_detector(output=True)
rospy.sleep(1)
rate = rospy.Rate(10)

circlefound=False
circlecount=0
vx=0
vy=0
yr=0
while not rospy.is_shutdown():
    vx=0
    vy=0
    yr=0
    hole_center=hole_detector.get_hole()
    space_center=hole_detector.get_hole()
    if circlefound==False and hole_center is not None:
        print(hole_center)
        yr=hole_center/4
        if -0.06<hole_center<0.06:
            circlecount+=1
            if circlecount>50:
                circlefound=True
    elif circlefound:
        vy=space_center/3
        vx=0.4
    else:
        yr=0.2
    target_pub.publish(construct_target(vx, vy, -1.5, yr))
    rate.sleep()
