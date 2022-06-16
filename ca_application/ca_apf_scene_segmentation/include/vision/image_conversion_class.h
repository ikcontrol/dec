/* 
 * DATE: 15/06/2022
 * 
 * AUTHOR: Diego Rodríguez <diego.rodriguez@ikerlan.es>
 * 
 * MANTAINER: Diego Rodríguez <diego.rodriguez@ikerlan.es>
 *            Josu Sinovas <jsinovas@ikerlan.es>
 * 
 * DESCRIPTION:
 * A class package for developing format conversion between ros and openCV. This class is used for grabbing images with RS D435 from ROS format and converting it
 * into openCV format. It will give the following images types: RGB, infra1, infra2, DepthMap, ComputedDisparityMap and ComputedPointCloud
 * 
*/
//////////////////////////////////////////////////////////////////////////////////////
///////////////////////////     DEFINE ZONE     //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
// #define                OPENCV_WINDOW                "Image Window"

#ifndef IMAGE_CONVERSION_CLASS_H
#define IMAGE_CONVERSION_CLASS_H

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////    INCLUDE ZONE    ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>                                    // ROS Library:
#include <ros/master.h>                                 // ROS Library: Master library to interact with ros master through code instead of topics
#include <sensor_msgs/Image.h>                          // ROS Library: Image type for sensors grabbed pictures
#include <sensor_msgs/image_encodings.h>                // ROS Library: Image encoding type are defined in this header file for ROS Image formats
#include <math.h>                                       // System Library:
#include <stdlib.h>                                     // System Library: 
#include <cv_bridge/cv_bridge.h>                        // Open CV 3 Library: to link ROS and OpenCV
#include <image_transport/image_transport.h>            // Open CV 3 Library: to manage picture images transformation
#include <opencv2/highgui/highgui.hpp>                  // Open CV 3 Library: to manage GUIs where to represent images
#include <opencv2/calib3d.hpp>                          // Open CV 3 Library: to load cv::StereoBM method

//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// CLASS DECLARATION ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
namespace pic_handling
{
    class image_conversion {
        private:
            /* Useful private attributes of the class */
            ros::Subscriber sColorSubs;                                                             // 
            ros::Subscriber sInfra1Subs;                                                            // 
            ros::Subscriber sInfra2Subs;                                                            // 
            ros::Subscriber sDepthSubs;                                                             // 

            bool _bAlignedDepth;                                                                    // _bAlignedDepth = true means that is aligned depth filter is on.

            sensor_msgs::Image imGrabbedColor, imGrabbedInfra1, imGrabbedInfra2, imGrabbedDepth;    // Image type message for ROS nodes communication. In this case, they represent the grabbed pictures by the RS D435 camera

            std::string sOpencvWindow;
            std::string sColorTopic, sInfra1Topic, sInfra2Topic, sDepthTopic;

            /* Useful private methods of the class */
            void init(std::string sCameraNumber = "", bool bAlignedDepthOn = true);

            /* ROS callback methods */
            void color_image_callback(const sensor_msgs::Image::ConstPtr &imageMsgPtr);
            void infra1_image_callback(const sensor_msgs::Image::ConstPtr &imageMsgPtr);
            void infra2_image_callback(const sensor_msgs::Image::ConstPtr &imageMsgPtr);
            void depth_image_callback(const sensor_msgs::Image::ConstPtr &imageMsgPtr);
            // void test_callback(const std_msgs::String::ConstPtr &stringMsgPtr);

        public:
            /* Constructor and destructor of the class */
            image_conversion(std::string sCameraNumber = "", bool bAlignedDepthOn = true);
            ~image_conversion();

            /* Public attributes of the class */

            /* Public methods of the class */
            bool displayImage(short int siNumImages=1);
            bool computeDisparityMapFromStereo();

            /* Setters and Getters */
            sensor_msgs::Image getimGrabbedColor();
            sensor_msgs::Image getimGrabbedInfra1();
            sensor_msgs::Image getimGrabbedInfra2();
            sensor_msgs::Image getimGrabbedDepth();
            cv::Mat getmColorImage();
            cv::Mat getmInfra1Image();
            cv::Mat getmInfra2Image();
            cv::Mat getmDepthImage();
            bool getbColorReceived();
            bool getbInfra1Received();
            bool getbInfra2Received();
            bool getbDepthReceived();
            bool getbColorOn();
            bool getbInfra1On();
            bool getbInfra2On();
            bool getbDepthOn();
            bool isAlignedDepthOn();
            void clearbColorReceived();
            void clearbInfra1Received();
            void clearbInfra2Received();
            void clearbDepthReceived();
        
        protected:
            /* Protected attributes of the class */
            ros::NodeHandle nh_;                                                                    // ROS node handler to attend the callback of the class

            bool bColorReceived, bInfra1Received, bInfra2Received, bDepthReceived;                  // Auxiliar boolean to check wheter or not an specific message type has been read
            bool bColorOn, bInfra1On, bInfra2On, bDepthOn;                                          // Auxiliar boolean to check if the subscription has been done or not. They not need a setter method

            cv::Mat mColorImage, mInfra1Image, mInfra2Image, mDepthImage;                           // Stored file for the grabbed images by ROS
            cv::Mat mComputedDisparityMap;                                                          // Computed disparity map

            /* Protected methods of the class */
    };
}


#endif // IMAGE_CONVERSION_CLASS_H