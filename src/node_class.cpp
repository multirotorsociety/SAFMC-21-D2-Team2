/**
 * @file offb_node.cpp
 * @brief Offboard control node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


class Drone
{
private:
  ros::NodeHandle nh;
  
  // ros msgs
  mavros_msgs::State current_state;
  mavros_msgs::PositionTarget current_target;
  geometry_msgs::PoseStamped current_pose;
  
  // ros subscribers
  ros::Subscriber state_sub;
  ros::Subscriber target_sub;
  ros::Subscriber local_pose_sub;

  // ros publishers
  ros::Publisher target_pub;

  // ros service clients
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;

public:
  Drone()
  {
    // ros subscribers
    state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, &Drone::state_cb, this);
    target_sub = nh.subscribe<mavros_msgs::PositionTarget>
        ("target", 10, &Drone::target_cb, this);
    local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, &Drone::local_pose_cb, this);
    
    // ros publishers
    target_pub = nh.advertise<mavros_msgs::PositionTarget>
        ("mavros/setpoint_raw/local", 10);

    // ros service clients
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

    // initialize current_target
    current_target.type_mask =
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_YAW |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    current_target.coordinate_frame = 
        mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;

    // set takeoff height
    current_target.position.z = 1.5;

    ROS_INFO("Created drone object!");
  }

  // callback functions
  void state_cb(const mavros_msgs::State::ConstPtr& msg)
  {
    /*if (current_state.armed != msg->armed)
    {
      ROS_INFO("armed state changed from %s to %s",
               current_state.armed, msg->armed);
    }
    if (current_state.connected != msg->connected)
    {
      ROS_INFO("connected state changed from %s to %s",
               current_state.connected, msg->connected);
    }
    if (current_state.mode != msg->mode)
    {
      ROS_INFO("mode changed from %s to %s", current_state.mode, msg->mode);
    }*/
    current_state = *msg;
  }

  void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    current_pose = *msg;
  }

  void target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
  {
    if (current_state.armed)
    {
      current_target = *msg;
    }
  }

  bool is_connected()
  {
    return current_state.connected;
  }

  bool is_armed()
  {
    return current_state.armed;
  }

  std::string get_mode()
  {
    return current_state.mode;
  }

  void publish_target()
  {
    target_pub.publish(current_target);
  }

  bool set_mode(mavros_msgs::SetMode *cmd)
  {
    return set_mode_client.call(*cmd);
  }

  bool set_arm(mavros_msgs::CommandBool *cmd)
  {
    return arming_client.call(*cmd);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offboard_node");
  Drone drone = Drone();
  ROS_INFO("starting");

  // publishing rate MUST be faster than 2Hz
  ros::Rate rate(20);
  
  // wait for FCU connection
  while(ros::ok() && !drone.is_connected())
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Connected!");

  //send a few setpoints before starting
  for(int i = 30; ros::ok() && i > 0; --i)
  {
    drone.publish_target();
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (drone.get_mode() != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      ROS_INFO("Setting offboard");
      if (drone.set_mode(&offb_set_mode) &&
          offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!drone.is_armed() &&
          (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        ROS_INFO("Arming");
        if (drone.set_arm(&arm_cmd) &&
            arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    //drone.publish_target();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}