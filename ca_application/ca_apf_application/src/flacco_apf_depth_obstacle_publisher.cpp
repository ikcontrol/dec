/*
Copyright (c) 2021, Ikerlan S. Coop.
All rights reserved.
*/

#include "ca_apf_application/flacco_apf_depth_obstacle_publisher.h"

#include <memory>
#include <vector>
#include <string>
#include <thread>  // NOLINT
#include <algorithm>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <tf2_eigen/tf2_eigen.h>

namespace app_apf_collision_avoidance
{

void DepthObstaclePublisher::init()
{
  apf_.cam_params = config_.cam_params;
  apf_.params.initial_ro = config_.ro;

  tf_listener_.reset(new tf2_ros::TransformListener(tf_));

  filtered_dimg_pub_ = it_.advertise("camera/post_proc_depth", 1);
  obstacles_pub_ = nh_.advertise<apf_msgs::ApfObstacles>("apf_dists", 10);
  ROS_INFO_STREAM("Publishing obstacle distance messages at '" << nh_.getNamespace() << "/apf_dists'");

  dimg_sub_ = it_.subscribe(config_.depth_img_topic, 1, &DepthObstaclePublisher::img_cb, this);
  ROS_INFO_STREAM("Listening for filtered depth images at '" << config_.depth_img_topic << "'");

  #ifdef OBSTACLES_PUB_DO_TIMINGS
  timer_ = ObstaclesPubTimer<OBSTACLES_PUB_TIMING_N>(OBSTACLES_PUB_TIMINGS_PATH);
  #endif
}

void DepthObstaclePublisher::set_config(obstacle_search_config cfg)
{
  config_ = cfg;
}

const obstacle_search_config& DepthObstaclePublisher::get_config() const
{
  return config_;
}

void DepthObstaclePublisher::img_cb(const sensor_msgs::ImageConstPtr& ros_depth_image)
{
  #ifdef OBSTACLES_PUB_DO_TIMINGS
  timer_.justCalled();
  #endif

  ROS_DEBUG("Received depth image!");
  // Map the image to eigne without copying the buffer
  cv::Mat cv_img;
  try
  {
    if (ros_depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      ROS_DEBUG("32FC1");
      auto orig_depth_img = cv_bridge::toCvShare(ros_depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
      cv_img = orig_depth_img->image;
    }
    else
    {
      ROS_DEBUG("16UC1");
      auto orig_depth_img = cv_bridge::toCvShare(ros_depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
      orig_depth_img->image.convertTo(cv_img, CV_32F, 0.001);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge Exception: %s", e.what());
    return;
  }

  {
    double min, max;
    cv::minMaxLoc(cv_img, &min, &max);
    ROS_DEBUG_STREAM("Depth in range " << min << " to " << max);
  }

  if (config_.do_post_proc)
  {
    post_proces_img(cv_img);
    auto img_msg = cv_bridge::CvImage(ros_depth_image->header, "32FC1", cv_img).toImageMsg();
    filtered_dimg_pub_.publish(img_msg);
  }

  #ifdef OBSTACLES_PUB_DO_TIMINGS
  timer_.postprocEnded();
  #endif

  Eigen::MatrixXd eig_img;
  cv::cv2eigen(cv_img, eig_img);

  ROS_DEBUG("Converted image to Eigen");

  // Create msg
  apf_msgs::ApfObstacles obs_msg;
  obs_msg.header = ros_depth_image->header;

  process_cps(config_.cps, ros_depth_image, eig_img, obs_msg);

  // Publish the msg
  obstacles_pub_.publish(obs_msg);
  ROS_DEBUG("Published msg");

  #ifdef OBSTACLES_PUB_DO_TIMINGS
  timer_.justEnded();
  #endif
}

void DepthObstaclePublisher::post_proces_img(cv::Mat& cv_img)
{
  auto& cfg = config_.postprocessing;

  // Filter image to remove outlier pixels
  cv::medianBlur(cv_img, cv_img, cfg.median_filter_k_size);  // Probably this should be a param

  // Threshold (detect background)
  cv::Mat th_img;
  cv::Mat th_img_8u;
  cv::threshold(cv_img, th_img, cfg.bg_threshold, 255, CV_THRESH_BINARY);
  th_img.convertTo(th_img_8u, CV_8U);

  // Start off current mask
  shadow_mask_ = th_img_8u.clone();

  // Or prev masks
  for (auto img : shadow_buffer_)
  {
    cv::bitwise_or(img, shadow_mask_, shadow_mask_);
  }

  // Keep last 3 frames
  shadow_buffer_.push_back(th_img_8u);
  if (shadow_buffer_.size() > cfg.frame_buffer_size)  // This should be a parameter
  {
    shadow_buffer_.pop_front();
  }

  auto close_se = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(cfg.close_k_size, cfg.close_k_size));
  cv::morphologyEx(shadow_mask_, shadow_mask_, cv::MORPH_CLOSE, close_se);

  auto dilate_se = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(cfg.dilate_k_size, cfg.dilate_k_size));
  cv::dilate(shadow_mask_, shadow_mask_, dilate_se);

  cv_img.setTo(cfg.replace_value, shadow_mask_);

  // Normalize image and convert to 8u for Canny edges
  cv::Mat norm_img;
  cv::Mat norm_img_8u;
  cv::normalize(cv_img, norm_img, 0, 255, cv::NORM_MINMAX);
  norm_img.convertTo(norm_img_8u, CV_8U);

  // Get edges
  cv::Mat blurred;
  cv::blur(norm_img_8u, blurred, cv::Size(3, 3));
  cv::Mat canny_edges;
  cv::Canny(blurred, canny_edges, cfg.canny_threshold[0], cfg.canny_threshold[1]);

  // Find contours in edges
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(canny_edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

  // Filter and render contours
  for (auto idx = 0; idx < contours.size(); idx++)
  {
    const auto& cnt = contours[idx];
    if (cv::contourArea(cnt) < cfg.min_contour_area)  // Config
    {
      cv::drawContours(cv_img, contours, idx, cv::Scalar(cfg.replace_value), CV_FILLED);
      continue;
    }
  }

  // Filter again for some more noise that might have been generated
  cv::medianBlur(cv_img, cv_img, cfg.median_filter_k_size);

  if (config_.postprocessing.gui)
  {
    cv::normalize(cv_img, norm_img, 0, 1.0, cv::NORM_MINMAX);
    cv::imshow("Display Window", norm_img);
    cv::imshow("MASK", shadow_mask_);
    cv::waitKey(1);
  }
}

void DepthObstaclePublisher::process_cps(const std::vector<std::string>& cps,
                                         const sensor_msgs::ImageConstPtr& ros_depth_image,
                                         const Eigen::MatrixXd& eig_img,
                                         apf_msgs::ApfObstacles& obs_msg)
{
  std::vector<std::thread> threads;

  obs_msg.control_points.resize(cps.size());
  int i = 0;
  for (std::string const& cp : cps)
  {
    threads.emplace_back(
      std::thread([this, &cp, i, &ros_depth_image, &eig_img, &obs_msg]() mutable
      {
        this->process_cp(cp, i, ros_depth_image, eig_img, obs_msg);
      }
    ));  // NOLINT
    ++i;
  }
  for (auto& t : threads)
  {
    t.join();
  }
}

void DepthObstaclePublisher::process_cp(const std::string& cp, int i,
                                        const sensor_msgs::ImageConstPtr& ros_depth_image,
                                        const Eigen::MatrixXd& eig_img,
                                        apf_msgs::ApfObstacles& obs_msg)
{
  ROS_DEBUG_STREAM("Processing control point: " << cp);
  // Get transform to camera frame and convert it to depth space in camera frame
  geometry_msgs::TransformStamped cart_tr;
  try
  {
    if (config_.loose_timing)
      cart_tr = tf_.lookupTransform(ros_depth_image->header.frame_id, cp, ros::Time(0));
    else
      cart_tr = tf_.lookupTransform(ros_depth_image->header.frame_id, cp, ros_depth_image->header.stamp);
  }
  catch(tf2::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  ROS_DEBUG("Obtained tf transform camera to cp");
  auto eig_cart_tr = tf2::transformToEigen(cart_tr);
  auto depth_tr = apf_.cart2Depth(eig_cart_tr.translation());
  ROS_DEBUG("Obtained depth space location of cp");

  // Compute distances to obstacles
  std::vector<Eigen::Vector3d> obs;
  if (config_.use_legacy_dist)
  {
    auto search_region = apf_.getSearchAreaRegion(depth_tr, config_.ro);
    apf_.getObstacleDistances(eig_img.cast<double>(), depth_tr, search_region, obs);
  }
  else
  {
    auto search_region = apf_.getSearchAreaRegion2(depth_tr, config_.ro);
    apf_.getObstacleDistances2(eig_img, depth_tr, search_region, obs);
  }

  ROS_DEBUG("Computed distances to obstacles from cp");
  // Copy to msg
  apf_msgs::CpApfObstacles cp_obs_msg;
  cp_obs_msg.cp_name = cp;
  for (const auto& o : obs)
  {
    geometry_msgs::Point p;
    p.x = o.x();
    p.y = o.y();
    p.z = o.z();
    cp_obs_msg.distances.push_back(p);
  }
  obs_msg.control_points[i] = cp_obs_msg;

  ROS_DEBUG("Added distances to msg");
}

bool read_cam_config(const ros::NodeHandle nh, obstacle_search_config& config)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  sleep(1);  // Sleep to allow tf buffer population

  const bool depth_img_topic_ok = nh.getParam("depth_image_topic", config.depth_img_topic);
  if (!depth_img_topic_ok)
  {
    ROS_ERROR_STREAM("Required parameter '" << nh.getNamespace() << "/depth_image_topic' not found");
    return false;
  }

  std::string camera_info_topic;
  const bool camera_ns_ok = nh.getParam("camera_info_topic", camera_info_topic);
  if (!camera_ns_ok)
  {
    ROS_ERROR_STREAM("Required parameter '" << nh.getNamespace() << "/camera_info_topic' not found");
    return false;
  }

  ROS_INFO_STREAM("Trying to obtain camera info from '" << camera_info_topic << "'...");
  sensor_msgs::CameraInfoConstPtr cam_info_ptr =
      ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic, ros::Duration(0));
  if (!cam_info_ptr)
  {
    ROS_ERROR_STREAM("Could not obtain camera info from '"
                     << camera_info_topic << "' after waiting for it");
    return false;
  }

  config.cam_params.K = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(cam_info_ptr->K.data());

  const std::string& camera_frame = cam_info_ptr->header.frame_id;
  std::string fixed_frame;
  const bool robot_frame_ok = nh.getParam("fixed_frame", fixed_frame);
  if (!robot_frame_ok)
  {
    ROS_ERROR_STREAM("Required parameter '" << nh.getNamespace() << "/fixed_frame' not found");
    return false;
  }

  geometry_msgs::TransformStamped tfRobot2Camera;
  try
  {
    ROS_INFO_STREAM("Waiting for transform between robot frame ('" << fixed_frame <<
                     "') and camera frame ('" << camera_frame << "')...");
    tfRobot2Camera = tfBuffer.lookupTransform(camera_frame, fixed_frame, ros::Time(0), ros::Duration(60.0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR_STREAM("Could not find transform between robot frame ('" << fixed_frame <<
                     "') and camera frame ('" << camera_frame << "') after 1 min: " << ex.what());
    return false;
  }

  auto eigRobot2Camera = tf2::transformToEigen(tfRobot2Camera);

  config.cam_params.t = eigRobot2Camera.translation();
  config.cam_params.R = eigRobot2Camera.rotation();

  return true;
}

bool read_postproc_config(const ros::NodeHandle nh, obstacle_search_config& config)
{
  {
    config.do_post_proc = true;
    const bool camera_ns_ok = nh.getParam("do_post_processing", config.do_post_proc);
    if (!config.do_post_proc)
      return true;
  }

  auto& pp = config.postprocessing;

  {
    const bool param_read_ok = nh.getParam("post_processing/bg_threshold", pp.bg_threshold);
    if (!param_read_ok)
    {
      ROS_ERROR_STREAM("Required parameter '" << nh.getNamespace() << "/post_processing/bg_threshold' not found");
      return false;
    }
  }

  {
    const bool param_read_ok = nh.getParam("post_processing/canny_threshold", pp.canny_threshold);
    if (!param_read_ok)
    {
      ROS_ERROR_STREAM("Required parameter '" << nh.getNamespace() << "/post_processing/canny_threshold' not found");
      return false;
    }
    if (pp.canny_threshold.size() != 2)
    {
      ROS_ERROR_STREAM("Required parameter '" << nh.getNamespace() << "/post_processing/canny_threshold' "
                       "should have 2 values");
      return false;
    }
  }

  {
    const bool param_read_ok = nh.getParam("post_processing/close_k_size", pp.close_k_size);
    if (!param_read_ok)
    {
      ROS_ERROR_STREAM("Required parameter '" << nh.getNamespace() << "/post_processing/close_k_size' not found");
      return false;
    }
  }

  {
    const bool param_read_ok = nh.getParam("post_processing/dilate_k_size", pp.dilate_k_size);
    if (!param_read_ok)
    {
      ROS_ERROR_STREAM("Required parameter '" << nh.getNamespace() << "/post_processing/dilate_k_size' not found");
      return false;
    }
  }

  {
    const bool param_read_ok = nh.getParam("post_processing/frame_buffer_size", pp.frame_buffer_size);
    if (!param_read_ok)
    {
      ROS_ERROR_STREAM("Required parameter '" << nh.getNamespace() << "/post_processing/frame_buffer_size' not found");
      return false;
    }
  }

  {
    const bool param_read_ok = nh.getParam("post_processing/median_filter_k_size", pp.median_filter_k_size);
    if (!param_read_ok)
    {
      ROS_ERROR_STREAM("Required parameter '" <<
                       nh.getNamespace() << "/post_processing/median_filter_k_size' not found");
      return false;
    }
  }

  {
    const bool param_read_ok = nh.getParam("post_processing/min_contour_area", pp.min_contour_area);
    if (!param_read_ok)
    {
      ROS_ERROR_STREAM("Required parameter '" << nh.getNamespace() << "/post_processing/min_contour_area' not found");
      return false;
    }
  }

  {
    const bool param_read_ok = nh.getParam("post_processing/replace_value", pp.replace_value);
    if (!param_read_ok)
    {
      ROS_ERROR_STREAM("Required parameter '" << nh.getNamespace() << "/post_processing/replace_value' not found");
      return false;
    }
  }

  {
    pp.gui = false;
    const bool param_read_ok = nh.getParam("post_processing/gui", pp.gui);
  }

  return true;
}

bool read_config(const ros::NodeHandle nh, obstacle_search_config& config)
{
  if (!read_cam_config(nh, config))
    return false;

  if (!read_postproc_config(nh, config))
    return false;

  XmlRpc::XmlRpcValue v;
  if (!nh.getParam("ro", v))
  {
    ROS_ERROR_STREAM("Could not find parameter '" << nh.getNamespace() << "/ro'");
    return false;
  }
  if (v.getType() !=  XmlRpc::XmlRpcValue::TypeDouble)
  {
    ROS_ERROR_STREAM("Parameter '" << nh.getNamespace() << "/ro' is not of the required type 'double'");
    return false;
  }
  config.ro = static_cast<double>(v);

  if (!nh.getParam("control_points", v))
  {
    ROS_ERROR_STREAM("Could not load parameter '" << nh.getNamespace() << "/control_points'");
    return false;
  }
  if (v.getType() !=  XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("Parameter '" << nh.getNamespace() << "/ro' is not of the required type 'double'");
    return false;
  }
  config.cps.clear();
  for (auto i = 0; i < v.size(); i++)
  {
    ROS_ASSERT(v[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    config.cps.push_back(static_cast<std::string>(v[i]));
  }

  nh.param("legacy_dist_calc", v, XmlRpc::XmlRpcValue(false));
  if (v.getType() !=  XmlRpc::XmlRpcValue::TypeBoolean)
  {
    ROS_ERROR_STREAM("Parameter '" << nh.getNamespace() << "/legacy_dist_calc' is not of the required type 'int'");
    return false;
  }
  config.use_legacy_dist = static_cast<bool>(v);

  nh.param("loose_timing", v, XmlRpc::XmlRpcValue(false));
  if (v.getType() !=  XmlRpc::XmlRpcValue::TypeBoolean)
  {
    ROS_ERROR_STREAM("Parameter '" << nh.getNamespace() << "/loose_timing' is not of the required type 'int'");
    return false;
  }
  config.loose_timing = static_cast<bool>(v);

  return true;
}

}  // namespace app_apf_collision_avoidance
