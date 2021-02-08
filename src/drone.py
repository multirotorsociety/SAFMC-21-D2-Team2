#!/usr/bin/env python3

import rospy
# import math
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
# from pymavlink import mavutil


class Drone:

    def __init__(self):
        self.arm_state = False
        self.offboard_state = False
        self.mav_type = None
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
        self.state_sub = rospy.Subscriber('mavros/state', State, \
            self.state_callback)

        '''
        ros publishers
        '''
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', \
            PositionTarget, queue_size=10)
        
        '''
        ros services
        '''
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flight_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.loginfo("Drone initialized!")

    # Callback functions
    def state_callback(self, data):
        ''' Debugging logging'''
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))
        '''
        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))
        '''
        self.state = data

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
                    res = self.arm_srv(True)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

                try:
                    res = self.flight_mode_srv(0, 'OFFBOARD')
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
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
        self.local_target_pub.publish(self.target_alt_msg)

    # Give velocity based on the drone frame
    def send_velocity(self, vel_x, vel_y, yaw_rate, duration):
        rospy.loginfo("Sending velocity command")
        self.target_vel_msg.velocity.x = vel_x
        self.target_vel_msg.velocity.y = vel_y
        self.target_vel_msg.yaw_rate = yaw_rate
        self.target_vel_msg.header.stamp = rospy.Time.now()
        self.local_target_pub.publish(self.target_vel_msg)

    # set land mode
    def set_land(self):
        rospy.loginfo("Landing")
        try:
            res = self.flight_mode_srv(0, 'AUTO.LAND')
            if not res.mode_sent:
                rospy.logerr("failed to send mode command")
        except rospy.ServiceException as e:
            rospy.logerr(e)

    # Disarm
    def disarm(self):
        rospy.loginfo("Disarming")
        try:
            res = self.arm_srv(False)
            if not res.success:
                rospy.logerr("failed to send disarm command")
        except rospy.ServiceException as e:
            rospy.logerr(e)

def main():
    rospy.init_node('drone_node', anonymous=True)
    drone = Drone()
    drone.arm_and_takeoff(1.5)
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