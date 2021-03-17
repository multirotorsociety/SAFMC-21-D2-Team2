#!/usr/bin/env python3

import rospy
import math
from mavros_msgs.msg import PositionTarget
from hole_detector import Hole_detector
from nav_msgs.msg import Odometry
from std_msgs.msg import String

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
    elif land:
        print("landing")
        land_pub.publish(String("LAND"))
    elif circlefound:
        print("moving forward")
        vy=-space_center/1.6
        vx=0.2
        yr=0
    else:
        yr=0.1
    if distance >4.2:
        land=True
    target_pub.publish(construct_target(vx, vy, z, yr))
    rate.sleep()
