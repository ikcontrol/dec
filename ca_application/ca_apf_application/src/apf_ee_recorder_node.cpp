/*
Copyright (c) 2021, Ikerlan S. Coop.
All rights reserved.
*/

#include <string>
#include <vector>
#include <fstream>

#include <sys/types.h>
#include <pwd.h>

#include <sys/mman.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <apf_controller_msgs/VelApfControllerState.h>

#include <ros/xmlrpc_manager.h>
#include <signal.h>

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1;
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

// APP FUNCTIONS
std::string id = "";
std::string getTimingLogFilename()
{
  auto uid = getuid();
  struct passwd* pw = getpwuid(uid);  // NOLINT(runtime/threadsafe_fn) This is single threaded

  if (pw == NULL)
  {
    std::cout << "getpwuid_r failed" << std::endl;
    return "ee_apf.csv";
  }

  std::string homedir(pw->pw_dir);
  return homedir + "/ee_apf_" + id + ".csv";
}

struct EEApfInfoEntry
{
  double t;
  double ee_x;
  double ee_y;
  double ee_z;
  double rep_x;
  double rep_y;
  double rep_z;
};

std::string apf_topic = "/vel_flacco_apf_controller/apf_status";
std::string base_frame = "base_link";
std::string ee_frame = "tool_center_point";

ros::Time start_time;

std::vector<EEApfInfoEntry> buffer;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::unique_ptr<tf2_ros::TransformListener> tfListener;

void constraintsCallback(const apf_controller_msgs::VelApfControllerStateConstPtr msg)
{
  // Get ee location from tf
  auto transform_ee_2_base = tfBuffer->lookupTransform(base_frame, ee_frame, ros::Time(0));
  auto transform_base_2_ee = tfBuffer->lookupTransform(ee_frame, base_frame, ros::Time(0));

  // Rotate rep vector to base_link frame
  const auto& q_msg = transform_ee_2_base.transform.rotation;
  tf2::Quaternion q_ee_2_base = tf2::Quaternion(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  auto rep = tf2::quatRotate(q_ee_2_base, tf2::Vector3(msg->v_rep.x, msg->v_rep.y, msg->v_rep.z));

  // Add to buffer
  EEApfInfoEntry e;
  e.rep_x = rep.x();
  e.rep_y = rep.y();
  e.rep_z = rep.z();
  e.ee_x = transform_ee_2_base.transform.translation.x;
  e.ee_y = transform_ee_2_base.transform.translation.y;
  e.ee_z = transform_ee_2_base.transform.translation.z;
  e.t = (transform_ee_2_base.header.stamp - start_time).toSec();
  buffer.push_back(e);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apf_ee_recorder", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigIntHandler);
  ros::NodeHandle n("~");

  n.getParam("id", id);

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  std::string path = getTimingLogFilename();

  tfBuffer.reset(new tf2_ros::Buffer());
  tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

  usleep(100000);  // Allow tf buffer to polulate

  start_time = ros::Time::now();

  auto sub = n.subscribe<apf_controller_msgs::VelApfControllerState>(apf_topic, 1, constraintsCallback);
  ROS_INFO_STREAM("Recording apf end effector data from " << apf_topic);
  while (!g_request_shutdown)
  {
    ros::spinOnce();
    usleep(20000);
  }

  ros::Duration duration = ros::Time::now() - start_time;
  ROS_INFO_STREAM("Saving ee apf data recorded during " << duration << "s to '" << path << "'");

  std::ofstream outFile(path);

  outFile << "ee_x,ee_y,ee_z,rep_x,rep_y,rep_z,t" << "\n";
  for (const auto& e : buffer)
  {
    outFile << e.ee_x << "," << e.ee_y << "," << e.ee_z<< ","
            << e.rep_x << "," << e.rep_y << "," << e.rep_z<< ","
            << e.t << "\n";
  }

  outFile.close();

  ros::shutdown();
  return 0;
}
