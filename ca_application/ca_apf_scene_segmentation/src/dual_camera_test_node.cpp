/*
 * Date: 15/04/2021
 * 
 * Author: Diego Rodríguez
 * 
 * Description:
 * A node for testing the dual camera launching in ROS environment
 * 
*/

//////////////////////////////////////////////////////////////
////////////////////// INCLUDE ZONE //////////////////////////
//////////////////////////////////////////////////////////////
#include "../include/vision/image_conversion_class.h"   // Self-Defined Class Library
// #include "../include/vision/image_processing_class.h"   // Self-Defined Class Library: For using vision processing functionalities
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
    ros::init(argc, argv, "environment_recognition_node");
    ros::NodeHandle nh;

    /* Instantiation of the ROS spinner to attend the call of ROS functions */
    ros::AsyncSpinner spinner(4);
    spinner.start();

    /* Local variable declaration */
    // pic_handling::image_conversion icObject;         // Object instantiation for one camera with no parameters
    pic_handling::image_conversion icCamera1("1");          // Object instantiation for dual camera setup with parameters for topic names
    pic_handling::image_conversion icCamera2("2");          // Object instantiation for dual camera setup with parameters for topic names

    /* ROS nodes subscribers and publishers declarations */

    /* Cyclic part of the main program */
    while(ros::ok()){
        /* One camera standalone test for a non parametriced constructor */
        // if(icObject.getbColorReceived()) {
        //     icObject.displayImage(1);
        //     icObject.clearbColorReceived();
        // }

        /* Dual camera connection test - for camera 1 */
        // if (icCamera1.getbColorReceived()) {
        //     icCamera1.displayImage(1);
        //     icCamera1.clearbColorReceived();
        // }
        if (icCamera1.getbDepthReceived()) {
            icCamera1.displayImage(4);
            icCamera1.clearbDepthReceived();
        }

        /* Dual camera connection test - for camera 2 */
        // if (icCamera2.getbColorReceived()) {
        //     icCamera2.displayImage(1);
        //     icCamera2.clearbColorReceived();
        // }
        if (icCamera2.getbDepthReceived()) {
            icCamera2.displayImage(4);
            icCamera2.clearbDepthReceived();
        }
    }
    ros::shutdown();
    return 0;
}
