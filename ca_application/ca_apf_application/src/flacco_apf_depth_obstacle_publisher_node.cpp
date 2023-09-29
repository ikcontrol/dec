/*
   Copyright 2023 IKERLAN S. Coop.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
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
