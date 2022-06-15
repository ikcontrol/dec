/* 
 * DATE: 15/06/2022
 * 
 * AUTHOR: Josu Sinovas <jsinovas@ikerlan.es>
 * 
 * DESCRIPTION:
 * A node that launches the modified scene segmentation algorithm to process in an advanced way the environment of the robot surroundings.
 * 
*/

#include "../include/vision/image_processing_class.h" 
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <thread>
#include <chrono>







int main(int argc, char** argv)
{
    /* ROS node initialization */
    ros::init(argc, argv, "image_processing_node");
    ros::NodeHandle nh;


    /* Instantiation of the ROS spinner to attend the call of ROS functions */
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ROS_INFO("Waiting for topic /camera1/color/image_raw to come up..");
    ros::topic::waitForMessage<sensor_msgs::Image>("/camera1/color/image_raw");
    pic_handling::image_processing imApp(nh);
    ROS_INFO("The image proccesing node has been finally started...");
  
    
    /* Launching the running mode of the application object */
    if(imApp.init()){
        ROS_INFO("All components has been initialized, launching the application");
        imApp.run();
    }
    else {

        ROS_ERROR("Initilization failed, exiting...");
        return 1;
    }


    /* Exiting ROS and closing the opened node */
    ROS_INFO("Exiting image processing node...");
    ros::shutdown();
    return 0;
}









    



  
