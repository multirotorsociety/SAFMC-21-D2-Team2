#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

land_pub = rospy.Publisher('/cmd_land', String, queue_size=10)
rospy.init_node('land_node', anonymous=True)

rate = rospy.Rate(2)
i = 0

while not rospy.is_shutdown():

    target_pub.publish(construct_target(vx, vy, z, yr))
    rate.sleep()
    i += 1
    if i > 10:
        break
