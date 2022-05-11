/*
Copyright (c) 2021, Ikerlan S. Coop.
All rights reserved.
*/

#ifndef APP_APF_COLLISION_AVOIDANCE_FLACCO_APF_DEPTH_OBSTACLE_PUBLISHER_H
#define APP_APF_COLLISION_AVOIDANCE_FLACCO_APF_DEPTH_OBSTACLE_PUBLISHER_H

#include <string>
#include <vector>
#include <deque>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <apf_msgs/ApfObstacles.h>

#include <apf_algorithms/flacco.h>

// #define OBSTACLES_PUB_DO_TIMINGS
#ifdef OBSTACLES_PUB_DO_TIMINGS
#include "app_apf_collision_avoidance/obstacle_pub_timing.h"
#endif  // OBSTACLES_PUB_DO_TIMINGS

namespace app_apf_collision_avoidance
{

struct post_proc_conf
{
  int median_filter_k_size;
  double bg_threshold;
  int frame_buffer_size;
  int close_k_size;
  int dilate_k_size;
  std::vector<int> canny_threshold;
  int min_contour_area;
  double replace_value;
  bool gui;
};

struct obstacle_search_config
{
  std::vector<std::string> cps;
  double ro;
  std::string fixed_frame;
  std::string depth_img_topic;
  apf_algorithms::CamParams cam_params;
  bool use_legacy_dist;
  bool loose_timing;
  bool do_post_proc;
  post_proc_conf postprocessing;
};

class DepthObstaclePublisher
{
public:
  explicit DepthObstaclePublisher(ros::NodeHandle nh) :
    nh_(nh), it_(nh) {}
  DepthObstaclePublisher(ros::NodeHandle nh, obstacle_search_config cfg) :
    nh_(nh), config_(cfg), it_(nh) {}
  ~DepthObstaclePublisher() = default;

  const obstacle_search_config& get_config() const;
  void set_config(obstacle_search_config cfg);

  void init();

private:
  obstacle_search_config config_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  #ifdef OBSTACLES_PUB_DO_TIMINGS
  ObstaclesPubTimer<OBSTACLES_PUB_TIMING_N> timer_;
  #endif  // OBSTACLES_PUB_DO_TIMINGS

  ros::Publisher obstacles_pub_;
  image_transport::Publisher filtered_dimg_pub_;
  image_transport::Subscriber dimg_sub_;

  apf_algorithms::ApfFlacco<6> apf_;
  tf2_ros::Buffer tf_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::deque<cv::Mat> shadow_buffer_;
  cv::Mat shadow_mask_;

  void img_cb(const sensor_msgs::ImageConstPtr& msg);
  void post_proces_img(cv::Mat& img);
  void process_cp(const std::string& cp, int i,
                  const sensor_msgs::ImageConstPtr& ros_depth_image,
                  const Eigen::MatrixXd& eig_img,
                  apf_msgs::ApfObstacles& obs_msg);
  void process_cps(const std::vector<std::string>& cps,
                   const sensor_msgs::ImageConstPtr& ros_depth_image,
                   const Eigen::MatrixXd& eig_img,
                   apf_msgs::ApfObstacles& obs_msg);
};

bool read_config(const ros::NodeHandle nh, obstacle_search_config& config);

}  // namespace app_apf_collision_avoidance

#endif  // APP_APF_COLLISION_AVOIDANCE_FLACCO_APF_DEPTH_OBSTACLE_PUBLISHER_H
