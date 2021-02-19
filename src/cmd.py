#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped

def construct_pos_target(x, y, z, yaw, yaw_rate=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = 9

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.type_mask = \
            PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
            + PositionTarget.FORCE

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose

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

def construct_vel_target(x, y, z):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = 8

        target_raw_pose.velocity.x = x
        target_raw_pose.velocity.y = y
        target_raw_pose.velocity.z = z

        target_raw_pose.type_mask = 3527

        return target_raw_pose

target_pub = rospy.Publisher('target', PositionTarget, queue_size=10)
rospy.init_node('cmd_node', anonymous=True)

while not rospy.is_shutdown():
    cmd = input('pos/vel/hyb x y z yaw: ')
    cmd = cmd.split()
    if cmd[0] == 'pos':
        target_pub.publish(construct_pos_target(float(cmd[1]), float(cmd[2]), float(cmd[3]), float(cmd[4])))
    elif cmd[0] == 'vel':
        target_pub.publish(construct_vel_target(float(cmd[1]), float(cmd[2]), float(cmd[3])))
    elif cmd[0] == 'hyb':
        target_pub.publish(construct_target(float(cmd[1]), float(cmd[2]), float(cmd[3]), float(cmd[4])))
    elif cmd[0] == 'q':
        break
