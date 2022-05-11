/*
Copyright (c) 2021, Ikerlan S. Coop.
All rights reserved.
*/

#include <string>
#include <vector>
#include <math.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <apf_msgs/ApfObstacles.h>
#include "ca_apf_application/flacco_apf_depth_obstacle_publisher.h"

int main(int argc, char **argv)
{
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  ros::init(argc, argv, "depth_space_obstacle_publisher");
  ros::NodeHandle n("obstacle_pub");
  ros::NodeHandle pn("~");

  app_apf_collision_avoidance::obstacle_search_config cfg;
  bool config_ok = app_apf_collision_avoidance::read_config(pn, cfg);
  if (!config_ok) return 1;

  ROS_INFO("Configured the obstacle publisher successfully");

  app_apf_collision_avoidance::DepthObstaclePublisher obs_pub(n, cfg);
  obs_pub.init();

  ros::spin();

  return 0;
}
