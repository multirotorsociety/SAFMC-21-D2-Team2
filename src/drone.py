#!/usr/bin/env python3

import rospy
# import math
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, Twist
from pyquaternion import Quaternion
import time


class Drone:

    def __init__(self):
        
        self.local_pose = None
        self.current_heading = None
        self.takeoff_height = 3.2

        self.arm_state = False
        self.offboard_state = False
        self.frame = "BODY"

        self.state = State()

        self.target_alt_msg = PositionTarget()
        self.target_vel_msg = PositionTarget()

        self.target_alt_msg.type_mask = \
            PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY \
            + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY \
            + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX \
            + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
            + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE \
            + PositionTarget.FORCE

        self.target_vel_msg.type_mask = \
            PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY \
            + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX \
            + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
            + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_PZ \
            + PositionTarget.FORCE

        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)

        '''
        ros publishers
        '''
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        
        '''
        ros services
        '''
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flight_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.loginfo("Drone initialized!")

    def start(self):
        #rospy.init_node("offboard_node")
        for i in range(10):
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)
        self.cur_target_pose = self.construct_target(0, 0, self.takeoff_height, self.q2yaw(self.local_pose.pose.orientation))

        #print ("self.cur_target_pose:", self.cur_target_pose, type(self.cur_target_pose))

        for i in range(10):
            self.local_target_pub.publish(self.cur_target_pose)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            time.sleep(0.2)


        if self.takeoff_detection():
            print("Vehicle Took Off!")

        else:
            print("Vehicle Took Off Failed!")
            return

        '''
        main ROS thread
        '''
        while self.arm_state and self.offboard_state and not rospy.is_shutdown():
            rate = rospy.Rate(10)
            self.local_target_pub.publish(self.cur_target_pose)

            if (self.state is "LAND") and (self.local_pose.pose.position.z < 0.15):
                if(self.disarm()):
                    self.state = "DISARMED"

            rate.sleep()

    def construct_target(self, x, y, z, yaw, yaw_rate=1):
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


    # Callback functions
    def mavros_state_callback(self, msg):
        ''' Debugging logging'''
        if self.state.armed != msg.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, msg.armed))

        if self.state.connected != msg.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, msg.connected))

        if self.state.mode != msg.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, msg.mode))
        '''
        if self.state.system_status != msg.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][msg.system_status].name))
        '''
        self.state = msg

    def local_pose_callback(self, msg):
        self.local_pose = msg

    
    # Helper functions
    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def arm(self):
        if self.arm_srv(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.arm_srv(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def offboard(self):
        if self.flight_mode_srv(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False

    def takeoff_detection(self):
        if self.local_pose.pose.position.z > 0.1 and self.offboard_state and self.arm_state:
            return True
        else:
            return False

    # Arm and takeoff to specific altitude
    def arm_and_takeoff(self, targetAltitude, timeout=10):
        # Arm:
        # self.local_target_pub.publish(self.target_alt_msg)
        rospy.loginfo("setting FCU arm")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)

        for i in range(timeout * loop_freq):
            if self.state.armed and self.state.mode == 'OFFBOARD':
                rospy.loginfo("set arm and mode success")
                break
            else:
                try:
                    res = self.flight_mode_srv(0, 'OFFBOARD')
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
                
                try:
                    res = self.arm_srv(True)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            rate.sleep()

        # Takeoff:
        rospy.loginfo("Taking off")
        try:
            res = self.flight_mode_srv(0, 'AUTO.TAKEOFF')
            if not res.mode_sent:
                rospy.logerr("failed to send mode command")
        except rospy.ServiceException as e:
            rospy.logerr(e)

    # Publishes target altitude
    def set_altitude(self, targetAltitude):
        rospy.loginfo("Setting altitude")
        self.target_alt_msg.position.z = targetAltitude
        self.target_alt_msg.header.stamp = rospy.Time.now()


    # Give velocity based on the drone frame
    def set_velocity(self, vel_x, vel_y, yaw_rate, duration):
        rospy.loginfo("Sending velocity command")
        self.target_vel_msg.velocity.x = vel_x
        self.target_vel_msg.velocity.y = vel_y
        self.target_vel_msg.yaw_rate = yaw_rate
        self.target_vel_msg.header.stamp = rospy.Time.now()


    # set land mode
    def set_land(self):
        rospy.loginfo("Landing")
        try:
            res = self.flight_mode_srv(0, 'AUTO.LAND')
            if not res.mode_sent:
                rospy.logerr("failed to send mode command")
        except rospy.ServiceException as e:
            rospy.logerr(e)


def main():
    rospy.init_node('drone_node', anonymous=True)
    drone = Drone()
    drone.start()
    #drone.arm_and_takeoff(1.5)
    #rospy.sleep(10)
    #drone.set_altitude(2.5)
    #rospy.sleep(10)
    #drone.set_land()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()