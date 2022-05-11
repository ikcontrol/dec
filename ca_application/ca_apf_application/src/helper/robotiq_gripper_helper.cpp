/************************************************************************
Owner: IKERLAN S. Coop.
Mantainer: Ander Gonzalez <ander.gonzalez@ikerlan.es>

Project description:
Helper class to make the calls to the robotiq gripper service.

Date: 25/9/2020
Version: 1.1

************************************************************************/
#include "helper/robotiq_gripper_helper.h"


robotiq_gripper_helper::robotiq_gripper_helper(ros::NodeHandle nh) :
target_force(255),
target_speed(255)
{
  _gripperSrvClient = nh.serviceClient<robotiq_gripper_msgs::GripperCommandSrv>("/robotiq_gripper_control/gripper_command");
}

robotiq_gripper_helper::~robotiq_gripper_helper(){}

bool robotiq_gripper_helper::close(robotiq_gripper_msgs::GripperCommandSrvResponse &response)
{
  return move_to(255, response);
}

bool robotiq_gripper_helper::open(robotiq_gripper_msgs::GripperCommandSrvResponse &response)
{
  return move_to(0, response);
}

bool robotiq_gripper_helper::reset(robotiq_gripper_msgs::GripperCommandSrvResponse &response)
{
  robotiq_gripper_msgs::GripperCommandSrv gSrv;
  gSrv.request.CommandType = gSrv.request.CMD_ACTIVATE;

  if (_gripperSrvClient.call(gSrv))
  {
    response = robotiq_gripper_msgs::GripperCommandSrvResponse(gSrv.response);
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service GripperCommandSrv");
    return false;
  }
}

bool robotiq_gripper_helper::get_state(robotiq_gripper_msgs::GripperCommandSrvResponse &response)
{
  robotiq_gripper_msgs::GripperCommandSrv gSrv;
  gSrv.request.CommandType = gSrv.request.CMD_GET_STATE;

  if (_gripperSrvClient.call(gSrv))
  {
    response = robotiq_gripper_msgs::GripperCommandSrvResponse(gSrv.response);
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service GripperCommandSrv");
    return false;
  }
}

bool robotiq_gripper_helper::move_to(uint8_t position, robotiq_gripper_msgs::GripperCommandSrvResponse &response)
{
  robotiq_gripper_msgs::GripperCommandSrv gSrv;
  gSrv.request.CommandType = gSrv.request.CMD_MOVE;
  gSrv.request.Force = target_force;
  gSrv.request.Velocity = target_speed;
  gSrv.request.Position = position;

  if (_gripperSrvClient.call(gSrv))
  {
    response = robotiq_gripper_msgs::GripperCommandSrvResponse(gSrv.response);
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service GripperCommandSrv");
    return false;
  }
}