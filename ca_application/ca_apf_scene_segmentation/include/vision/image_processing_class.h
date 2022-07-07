/* 
 * DATE: 15/06/2022
 * 
 * AUTHOR: Diego Rodríguez <diego.rodriguez@ikerlan.es>
 * 
 * MANTAINER: Diego Rodríguez <diego.rodriguez@ikerlan.es>
 *            Josu Sinovas <jsinovas@ikerlan.es>
 * 
 * DESCRIPTION:
 * A derived class definition from image conversion class. The goal of this new class is to use the information received from ROS images transformed into OpenCV format. Thus,
 * the processing of the image and the relevant information about the picture can be obtained.
 * 
*/
//////////////////////////////////////////////////////////////////////////////////////
///////////////////////////     DEFINE ZONE     //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
#ifndef IMAGE_PROCESSING_CLASS_H
#define IMAGE_PROCESSING_CLASS_H
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////    INCLUDE ZONE    ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
#include "image_conversion_class.h"                     // IMAGE_CONVERSION_CLASS_H: it contains the parent class needed

#include <ros/ros.h>                                    // ROS Library:
#include <ros/master.h>                                 // ROS Library: Master library to interact with ros master through code instead of topics

#include <sensor_msgs/Image.h>                          // ROS Library: Image type for sensors grabbed pictures
#include <sensor_msgs/image_encodings.h>                // ROS Library: Image encoding type are defined in this header file for ROS Image formats

#include <math.h>                                       // System Library:
#include <stdlib.h>                                     // System Library: 

#include <cv_bridge/cv_bridge.h>                        // Open CV 3 Library: to link ROS and OpenCV
#include <image_transport/image_transport.h>            // Open CV 3 Library: to manage picture images transformation
#include <opencv2/highgui/highgui.hpp>                  // Open CV 3 Library: to manage GUIs where to represent images
#include <opencv2/imgproc.hpp>                          // Open CV 3 Library: to manage image procesing
// #include <opencv2/imgcodecs.hpp>                        // Open CV 3 Library: to manage image codification
// #include <opencv2/videoio.hpp>                          // Open CV 3 Library: to manage video I/O
#include <opencv2/video.hpp>                            // Open CV 3 Library: to manage video data (it contains background substraction methods)

#include <tf2_ros/transform_listener.h>                 // ROS Library: used for transforming between frames in point cloud operations
#include <tf2_ros/transform_broadcaster.h>              // ROS Library: for dynamic broadcasting of frames using tf2
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Quaternion.h>                  // ROS Library: to use quaternion type messages in ROS environment
#include <geometry_msgs/TransformStamped.h>             // ROS Library: to loead message type to use transformed stamped ROS messages

#include <sensor_msgs/PointCloud2.h>  
#include <tf/transform_listener.h>                  // ROS Library; for loading and using PointCloud 2 to publish the obstacles information
// #include <pcl_ros/point_cloud.h>                        // ROS Library: for PCL libraries usage
// #include <pcl_ros/transforms.h>                         // ROS Library: for transforms coordination between point clouds and ROS
// // #include <pcl/point_types.h>                            // ROS Library: for loading the different point clouds types avialables
// #include <pcl_conversions/pcl_conversions.h>            // ROS Library: for allowing conversions between ROS messages PointCloud types and the standard ones
// #include <pcl/filters/voxel_grid.h>                     // PCL Library: to load voxel grid filtering methods



#include <string>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>

#include <sensor_msgs/CameraInfo.h>    






/////////////////////////////////////////////////////////////////////////////////////
//////////////// SELF-DEFINED STRUCTS IMPLEMENTATION ////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
struct obstacle_parameters {
    double dClosestPoint;
    double dFurthestPoint;
    double dWidthCenter;
    double dHeightCenter;
    double dWidthLower;
    double dWidthHigher;
    double dHeightLower;
    double dHeightHigher;
};

struct obstacle_data {
    short int siObsFound;
    std::vector<obstacle_parameters> vopObsParam;
};

struct obs_cartheseian_components {
    double dXcenter;
    double dYcenter;
    double dZcenter;
    double dMinCartRadius;
    double dXdiff;
    double dYdiff;
    double dZdiff;
};

struct obs_carthesian_position {
    short int siNumberObstacles;
    std::vector<short int> vsiPublishCamera;
    std::vector<obs_cartheseian_components> occObstaclePoses;
};

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// CLASS DECLARATION ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
namespace pic_handling 
{
    class image_processing {
        private:
            tf::TransformListener listener;                                                         // Instantiation of TF listener
            tf::StampedTransform base_0;                                                            // Instantiation of Base TF
            tf::StampedTransform forearm;                                                           // Instantiation of Forearm TF
            tf::StampedTransform shoulder;                                                          // Instantiation of shoulder TF
            tf::StampedTransform wrist_1;                                                           // Instantiation of wrist_1 TF
            tf::StampedTransform wrist_2;                                                           // Instantiation of wrist_2 TF
            tf::StampedTransform wrist_3;                                                           // Instantiation of wrist_3 TF
            tf::StampedTransform tool_center_point;                                                 // Instantiation of tool_center_point TF
            int ImageNumber;                                                                        // Instantiation of the number of image 
            double z_imageBase,x_imageBase,y_imageBase;                                             // Instantiation of the position of the Base
            double z_imageforearm,x_imageforearm,y_imageforearm;                                    // Instantiation of the position of the Forearm
            double z_imageshoulder,x_imageshoulder,y_imageshoulder;                                 // Instantiation of the position of the Shoulder
            double z_imagewrist_1,x_imagewrist_1,y_imagewrist_1;                                    // Instantiation of the position of the Wrist_1
            double z_imagewrist_2,x_imagewrist_2,y_imagewrist_2 ;                                   // Instantiation of the position of the Wrist_2
            double z_imagewrist_3,x_imagewrist_3,y_imagewrist_3;                                    // Instantiation of the position of the Wrist_3
            double z_imagetool_center_point,x_imagetool_center_point,y_imagetool_center_point;      // Instantiation of the position of the Tool_center_point
            double z_imageforearm_real,x_imageforearm_real,y_imageforearm_real;                     // Instantiation of the position of the other side of the Forearm
            double z_imageshoulder_real,x_imageshoulder_real,y_imageshoulder_real;                  // Instantiation of the position of the other side of the Shoulder
            double z_imagewrist_1_real;
            double x_imagewrist_1_real;
            double y_imagewrist_1_real;
            
            
            cv::Mat ImageProf,ImageProf2;                                                           // Instantiation of the U16 depth image (2->new, _->old)
            cv::Mat ImagenProf_U8,ImagenProf2_U8;                                                   // Instantiation of the U8 depth image
            cv::Mat ImageRGB,ImageRGB2;                                                             // Instantiation of RGB image (2->new, _->old)
            cv::Mat SegRobot;                                                                       // Instantiation of the segmentation of the robot
            cv::Mat BlackImage;                                                                     // Instantiation of a Black image
            cv::Mat ResFinSeg;                                                                      // Instantiation of the colour difference (ImageRGB,ImageRGB2)
            cv::Mat forearm_wrist_1;                                                                // Instantiation of supposed forearm_wrist_1 image
            cv::Mat shoulder_forearm_real;                                                          // Instantiation of supposed shoulder_forearm_real image
            cv::Mat wrist_1_wrist_2;                                                                // Instantiation of supposed wrist_1_wrist_2 image
            cv::Mat wrist_2_wrist_3;                                                                // Instantiation of supposed wrist_2_wrist_3 image
            cv::Mat wrist_3_tool_center_point;                                                      // Instantiation of supposed wrist_3_tool_center_point image
            cv::Mat Prof_No_Robot;                                                                  // Instantiation of depth image without robot
            cv::Mat Prof_No_Robot_Old;                                                              // Instantiation of old depth image without robot
            cv::Mat Image_Prof_Background;                                                          // Instantiation of depth image Background
            cv::Mat SegNoRobot;                                                                     // Instantiation of segmentation without robot
            cv::Mat Objects;                                                                        // Instantiation of Objects detected
            cv::Mat Shadow_Image;                                                                   // Instantiation of Shadow detected
            cv::Mat Prof_seg_Image;                                                                 // Instantiation of depth segmentation
            cv::Mat Objects_scene,Objects_scene2,Objects_scene3,Objects_scene4;                     // Instantiation of the last 4 Object scene detected
            cv::Mat static_object;                                                                  // Instantiation of static objects
            cv::Mat dynamic_object;                                                                 // Instantiation of dynamic objects
            std::vector<cv::Mat> ObjectsSceneBuffer;                                                // Instantiation of the buffer for identify static objects
            int BufferSize;                                                                         // Instantiation of the size of the ObjectsSceneBuffer
            std::string sHomeDir;                                                                   // It stores the value of $HOME environment variable

            
            void segmentation ();                                                                      // obtain RGB frame difference  (ResFinSeg)
            void obtain_objects_RGB();                                                               // substract reference robot position from ResFinSeg (SegRobot)  
            void link_space(float x1_rect,float y1_rect,float x2_rect,float y2_rect,cv::Mat ImagenSolucion ); // obtain link space
            void Calculate_Tf();                                                                    // Calculate the joints position
            void RemoveRobot();                                                                     // get the depth filter pic(SegNoRobot) and obtain the objects (Objects)  
            void shadow();                                                                          // substract shadow from objects (Objects)
            void Prof_segmentation();                                                               // obtain frame difference depth filter pic and add to Objects (Objects_scene)  
            void object_contour(int cont);                                                          //draw the contour of dynamic and static objects in RGB pic
            void diff_static_dynamic(int cont);                                                     // identify static and dynamic objets 
            void publish_objects(int cont);                                                         // publish static, dynamic and scene segmentation

            

            
            






            /* Useful private attributes of the class */
            ros::NodeHandle _nh;                                                // ROS node handler

            ros::Publisher static_pub;                                          // published static objects pic 
            ros::Publisher dynamic_pub;                                         // published dynamic objects pic 
            ros::Publisher scene_pub;                                           // published scene segmentation pic (all static objects in the scene)
            ros::Publisher static_pub_info;                                     // published static objects pic info 
            ros::Publisher dynamic_pub_info;                                    // published dynamic objects pic info
            ros::Publisher scene_pub_info;                                      // published scene segmentation pic info


            bool bImageProcessingAppRunning = false;                            // Attribute to store wheter the image_processing class is running or not
            bool bColorStored1, bColorStored2, bDepthStored1, bDepthStored2;    // Attributes to store whether or not the camera received images has been stored
            bool bIsProcessing;                                                 // Attribute to check whether or not the main_loop() is busy processing the images or not

            image_conversion icCamGrabber1;                                     // Object instantiation for managing the first camera
            image_conversion icCamGrabber2;                                     // Object instantiation for manging the second camera pictures of the algorithm

            cv::Mat mColorPic1, mColorPic2, mDepthPic1, mDepthPic2;             // Instantiation of the stored color and depth pics to work with
            cv::Mat mDepthDistance1, mDepthDistance2;                           // Instantiation of the objects to store the distance pictures matrix.
            cv::Mat mDepthColor1, mDepthColor2;                                 // Instantiation of the objects to store the colour_jet distance pictures.
            cv::Mat mColorProcessed1, mColorProcessed2;                         // Instantiation of the objects to store the processed image of color cameras


            sensor_msgs::CameraInfoConstPtr ciColorInfoPtr1, ciColorInfoPtr2;   // Instantiation of sensor_msgs::CameraInfoConstPtr attributes to check wheter the topics are being published or not.
            sensor_msgs::CameraInfo ciColorInfo1, ciColorInfo2;                 // Instantiation of sensor_msgs::CameraInfo attributes to store the calibration parameters of the camera.
            sensor_msgs::CameraInfoConstPtr ciDepthInfoPtr1, ciDepthInfoPtr2;   // Instantiation of sensor_msgs::CameraInfoConstPtr attributes to check wheter the topics are being published or not.
            sensor_msgs::CameraInfo ciDepthInfo1, ciDepthInfo2;                 // Instantiation of sensor_msgs::CameraInfo attributes to store the calibration parameters of the camera.
            double dCam1_cx, dCam1_fx, dCam1_cy, dCam1_fy;                      // Instantiation of the attributes to store the extrinsics of the color camera 1 calibration
            double dCam2_cx, dCam2_fx, dCam2_cy, dCam2_fy;                      // Instantiation of the attribtues to store the extrinsics of the color camera 2 calibration
            double dCamD1_cx, dCamD1_fx, dCamD1_cy, dCamD1_fy;                  // Instantiation of the attributes to store the extrinsics of the depth camera 1 calibration
            double dCamD2_cx, dCamD2_fx, dCamD2_cy, dCamD2_fy;                  // Instantiation of the attributes to store the extrinsics of the depth camera 2 calibration 



            /* Useful private methods of the class */
            void main_loop();                                                   // Private method to encapsulate the cyclic functionalities of the image processing algorithm
            

            /* ROS callback methods */

        public:
            /* Constructor and destructor of the class */
            image_processing(ros::NodeHandle nh);                               // Constructor of the class               
            ~image_processing();                                                // Destructor of the class

            /* Public attributes of the class */

            /* Public methods of the class */
            bool init();                                                        // Initialization routine method
            void run();                                                         // Running routine method

            /* Setters and Getters */
            bool getbIsProcessing();                                            // Returns if the processing of the picture is being done or not
            void showCam1ColorPic();                                            // Returns the attribute mColorPic1 (raw images taken by the RSD435)
            void showCam2ColorPic();                                            // Returns the attribute mColorPic2 (raw images taken by the RSD435)
            void showCam1DepthPic();                                            // Returns the attribute mDepthPic1 (raw images taken by the RSD435)
            void showCam2DepthPic();                                            // Returns the attribute mDepthPic2 (raw images taken by the RSD435)
            void showCam1ColorProcessedPic();                                   // Returns the attribute mColorProcessed1 (processed image for camera 1 after detecting the obstacles)
            void showCam2ColorProcessedPic();                                   // Returns the attribute mColorProcessed2 (processed image for camera 2 after detecting the obstacles)
            cv::Scalar getsLowerThreshold();                                    // Returns the value of the lower threshold for an HSV filter
            cv::Scalar getsUpperThreshold();                                    // Returns the value of the upper threshold for an HSV filter
            double getdAreaThreshold();                                         // Returns the value of the area threshold for deleting the non-obstacle detected noise
            double getdDistanceThreshold();                                     // Returns the distance threshold in meters to check the matching of sides and centers between pics of camera 1 and camera 2
            int getErosioniKernelSize();                                        // Returns the size of the erosion kernel used
            sensor_msgs::CameraInfo getCam1ColorInfo();                         // Returns the information of the ROS color camera 1 configuration
            sensor_msgs::CameraInfo getCam2ColorInfo();                         // Returns the information of the ROS color camera 2 configuration
            sensor_msgs::CameraInfo getCam1DepthInfo();                         // Returns the information of the ROS depth camera 1 configuration
            sensor_msgs::CameraInfo getCam2DepthInfo();                         // Returns the information of the ROS depth camera 2 configuration
            obs_carthesian_position getObstaclesXYZPositions();                 // Returns the XYZ, minimum carthesian radius, and XYZ_diff for each of the detected obstacles


            bool setsLowerThreshold(cv::Scalar sThreshValue);                   // Changes the stored threshold value for lower HSV filter with a new one, given by the user
            bool setsUpperThreshold(cv::Scalar sThreshValue);                   // Changes the stored threshold value for upper HSV filter with a new one, given by the user
            bool setdAreaThreshold(double dAreaValue);                          // Changes the stored value of the area threshold for filtering non-obstacles detected noise
            bool setdDistanceThreshold(double dDistanceValue);                  // Changes the stored value of the distance threshold to check the matching of sides and center between camera 1 and camera 2 pictures
            bool setErosioniKernelSize(int iKernelValue);                       // Changes the stored value of the kernel size for the erosion process

        protected:
            /* Protected attributes of the class */

            /* Protected methods of the class */
    };
}


#endif // IMAGE_PROCESSING_CLASS_H