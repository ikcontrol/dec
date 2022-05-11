/*
 * Date: 09/10/2020
 * 
 * Author: Diego Rodríguez
 * 
 * Description:
 * A node for vision processing is developed in this file.
 * 
*/

//////////////////////////////////////////////////////////////
////////////////////// INCLUDE ZONE //////////////////////////
//////////////////////////////////////////////////////////////
// #include "../include/vision/image_conversion_class.h"   // Self-Defined Class Library
#include "../include/vision/image_processing_class.h"   // Self-Defined Class Library: For using vision processing functionalities
#include <ros/ros.h>                                    // ROS Librarry:
#include <math.h>                                       // System Library:
#include <stdio.h>                                      // System Library: 
#include <cv_bridge/cv_bridge.h>                        // Open CV 3 Library: to link ROS and OpenCV
#include <image_transport/image_transport.h>            // Open CV 3 Library: to manage picture images transformation
#include <opencv2/opencv.hpp>                           // Open CV 3 Library

//////////////////////////////////////////////////////////////
///////////// GLOBAL VARIABLES DECLARATION ///////////////////
//////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
//////////// LOCAL FUNCTIONS DECLARATIONS ////////////////////
//////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////
///////////// MAIN PROGRAM DECLARATION ///////////////////////
//////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    /* Initialization of a ROS node and its handler */
    ros::init(argc, argv, "image_processing_node");
    ros::NodeHandle nh;

    /* Instantiation of the ROS spinner to attend the call of ROS functions */
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ROS_INFO("The image proccesing node has been started...");
    ros::Duration(8.5).sleep();

    /* Local variable declaration */
    pic_handling::image_processing imApp(nh);
    
    /* ROS nodes subscribers and publishers declarations */

    /* Cyclic part of the main program */
    if(imApp.init()){
        ROS_INFO("All components has been initialized, launching the application");
        imApp.run();
    }
    else {
        ROS_ERROR("Initilization failed, exiting...");
        return 1;
    }

    ROS_INFO("Exiting image processing node...");
    ros::shutdown();
    return 0;
}
