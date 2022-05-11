/************************************************************************
Owner: IKERLAN S. Coop.
Mantainer: Ander Gonzalez <ander.gonzalez@ikerlan.es>

Project description:
Helper class to make the calls to the robotiq gripper service.

Date: 25/9/2020
Version: 1.1

************************************************************************/
#ifndef ROBOTIQ_GRIPPER_HELPER_H
#define ROBOTIQ_GRIPPER_HELPER_H

#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <robotiq_gripper_msgs/GripperCommandSrv.h>

class robotiq_gripper_helper
{
public:
  robotiq_gripper_helper(ros::NodeHandle nh);
  ~robotiq_gripper_helper();

  bool move_to(uint8_t position, robotiq_gripper_msgs::GripperCommandSrvResponse& response);
  bool close(robotiq_gripper_msgs::GripperCommandSrvResponse& response);
  bool open(robotiq_gripper_msgs::GripperCommandSrvResponse &response);
  bool get_state(robotiq_gripper_msgs::GripperCommandSrvResponse &response);
  bool reset(robotiq_gripper_msgs::GripperCommandSrvResponse &response);

  uint8_t target_speed;
  uint8_t target_force;


private:
  ros::ServiceClient _gripperSrvClient;
};

#endif //ROBOTIQ_GRIPPER_HELPER_H
