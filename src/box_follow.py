#!/usr/bin/env python3

import rospy
import math
from mavros_msgs.msg import PositionTarget
from box import BoxCV

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
rospy.init_node('cmd_node', anonymous=True)

box = BoxCV(output=True)
rospy.sleep(3)
rate = rospy.Rate(10)

px = 0.25
pyaw = 0.25

while not rospy.is_shutdown():
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
    
    target_pub.publish(construct_target(vx, -0.15, 1.1, yr))
    rate.sleep()
