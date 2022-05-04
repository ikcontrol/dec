/* 
 * DATE: 21/12/2020
 * 
 * AUTHOR: Diego Rodríguez <diego.rodriguez@ikerlan.es>
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

#include <sensor_msgs/PointCloud2.h>                    // ROS Library; for loading and using PointCloud 2 to publish the obstacles information
#include <pcl_ros/point_cloud.h>                        // ROS Library: for PCL libraries usage
#include <pcl_ros/transforms.h>                         // ROS Library: for transforms coordination between point clouds and ROS
#include <pcl/point_types.h>                            // ROS Library: for loading the different point clouds types avialables
#include <pcl_conversions/pcl_conversions.h>            // ROS Library: for allowing conversions between ROS messages PointCloud types and the standard ones
#include <pcl/filters/voxel_grid.h>                     // PCL Library: to load voxel grid filtering methods

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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// CLASS DECLARATION ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
namespace pic_handling 
{
    class image_processing {
        private:
            /* Useful private attributes of the class */
            ros::NodeHandle _nh;                                                // ROS node handler
            ros::Publisher pcl_pub;                                             // ROS publisher object to publish in the filtered point cloud topic
            ros::Publisher pcl_img_pub, pcl_info_pub;                           // TODO: delete or check accuracy

            bool bImageProcessingAppRunning = false;                            // Attribute to store wheter the image_processing class is running or not
            bool bColorStored1, bColorStored2, bDepthStored1, bDepthStored2;    // Attributes to store whether or not the camera received images has been stored
            bool bIsProcessing;                                                 // Attribute to check whether or not the main_loop() is busy processing the images or not

            image_conversion icCamGrabber1;                                     // Object instantiation for managing the first camera
            image_conversion icCamGrabber2;                                     // Object instantiation for manging the second camera pictures of the algorithm

            cv::Mat mColorPic1, mColorPic2, mDepthPic1, mDepthPic2;             // Instantiation of the stored color and depth pics to work with
            cv::Mat mDepthDistance1, mDepthDistance2;                           // Instantiation of the objects to store the distance pictures matrix.
            cv::Mat mDepthColor1, mDepthColor2;                                 // Instantiation of the objects to store the colour_jet distance pictures.
            cv::Mat mColorProcessed1, mColorProcessed2;                         // Instantiation of the objects to store the processed image of color cameras

            cv::Scalar sLowerThreshold, sUpperThreshold;                        // Instantiation of the cv::Scalar threshold for filtering only green obstacles

            double dAreaThreshold;                                              // Area threshold attribute 
            double dDistanceThreshold;                                          // Distance thresholde in [m] for computing the obstacles matching between cam1 and cam2
            int iKernelSize;                                                    // Kernel size for eroding the pictures. Another allowed value might be 18
            int iSceneMin, iSceneMax;                                           // Minimum threshold and maximum threshold for point cloud processing
            std::string sPubFrame1, sPubFrame2;                                 // Strings that contain the frame name of the publishing where the PointCloud is being published


            sensor_msgs::CameraInfoConstPtr ciColorInfoPtr1, ciColorInfoPtr2;   // Instantiation of sensor_msgs::CameraInfoConstPtr attributes to check wheter the topics are being published or not.
            sensor_msgs::CameraInfo ciColorInfo1, ciColorInfo2;                 // Instantiation of sensor_msgs::CameraInfo attributes to store the calibration parameters of the camera.
            sensor_msgs::CameraInfoConstPtr ciDepthInfoPtr1, ciDepthInfoPtr2;   // Instantiation of sensor_msgs::CameraInfoConstPtr attributes to check wheter the topics are being published or not.
            sensor_msgs::CameraInfo ciDepthInfo1, ciDepthInfo2;                 // Instantiation of sensor_msgs::CameraInfo attributes to store the calibration parameters of the camera.
            double dCam1_cx, dCam1_fx, dCam1_cy, dCam1_fy;                      // Instantiation of the attributes to store the extrinsics of the color camera 1 calibration
            double dCam2_cx, dCam2_fx, dCam2_cy, dCam2_fy;                      // Instantiation of the attribtues to store the extrinsics of the color camera 2 calibration
            double dCamD1_cx, dCamD1_fx, dCamD1_cy, dCamD1_fy;                  // Instantiation of the attributes to store the extrinsics of the depth camera 1 calibration
            double dCamD2_cx, dCamD2_fx, dCamD2_cy, dCamD2_fy;                  // Instantiation of the attributes to store the extrinsics of the depth camera 2 calibration 

            obstacle_data odZYCam1;                                             // Attribute to store the number of obstacles found in camera 1 picture
            obstacle_data odXYCam2;                                             // Attribute to store the number of obstacles found in camera 2 picture
            obs_carthesian_position ocpXYZPositions;                            // Attribute to store the XYZ positions of the detected obstacles

            sensor_msgs::PointCloud2 pcObsPointCloud;                           // Attribute to store an unique point cloud to store the different filtered point clouds by each camera

            /* Useful private methods of the class */
            void main_loop();                                                   // Private method to encapsulate the cyclic functionalities of the image processing algorithm
            obstacle_data process_obstacles_zy_plane();                         // Method to process the given cv::Mat from camera 1 and return the number of detected obstacles as well as the positioning box, if any.
            obstacle_data process_obstacles_xy_plane();                         // Method to process the given cv::Mat from camera 2 and return the number of detected obstacles as well as their position, if any. 
            obs_carthesian_position compute_xyz_obstacle_position();            // Method to compute the xyz position and minimum safety radius for each detected obstacle       
            obs_carthesian_position regular_xyz_computation();                  // Auxiliar method to compute obs_carthesian_position of all obstacles when the number of detected obstacles in camera 1 matches with the ones detected in camera 2
            obs_carthesian_position hard_occlusion_xyz_computation();           // Auxiliar method to compute obs_carthesian position of all obstacles when a case of hard occlusion is detected (one view completelly occlusioned or obstacle split)
            bool publish_stamped_obs_poses(obs_carthesian_position ocpVar);     // Auxiliar method for debugging and publishing the obstacle stamped position into ROS environment
            bool publish_obs_point_cloud();                                     // Method to publish the general computed point cloud that mixes both camera filtered depth images

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