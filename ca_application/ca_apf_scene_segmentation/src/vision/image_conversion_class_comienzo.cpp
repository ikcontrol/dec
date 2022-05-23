#include "../../include/vision/image_conversion_class.h"

namespace pic_handling
{
    /***********************************************************
     * PRIVATE METHODS OF THE CLASS
     **********************************************************/ 
    void image_conversion::init(std::string sCameraNumber, bool bAlignedDepthOn) {
        std::string sOutMessage;
        /* Configuring topic string names and opencv2 window names in case there's more than one camera --> Given de sCameraNumber parameter */
        if (sCameraNumber != "") {
            sOpencvWindow = "Image Window - RealSense camera" + sCameraNumber;
            sColorTopic = "/camera" + sCameraNumber + "/color/image_raw";
            sInfra1Topic = "/camera" + sCameraNumber + "/infra1/image_rect_raw";
            sInfra2Topic = "/camera" + sCameraNumber + "/infra2/image_rect_raw";
            if (bAlignedDepthOn) {
                /* Aligned depth is on -> So, we are going to record the depth picture preprocessed to match with the pixels and size of the rgb picture */
                sDepthTopic = "/camera" + sCameraNumber + "/aligned_depth_to_color/image_raw";
            }
            else {
                /* Aligned depth is off -> So, we are going to record the depth picture without the correction to match with the rgb picture */
                sDepthTopic = "/camera" + sCameraNumber + "/depth/image_rect_raw";
            }
        }
        else {
            sOpencvWindow = "Image Window - RealSense camera";
            sColorTopic = "/camera/color/image_raw";
            sInfra1Topic = "/camera/infra1/image_rect_raw";
            sInfra2Topic = "/camera/infra2/image_rect_raw";
            if (bAlignedDepthOn) {
                /* Aligned depth is on -> So, we are going to record the depth picture preprocessed to match with the pixels and size of the rgb picture */
                sDepthTopic = "/camera/aligned_depth_to_color/image_raw";
            }
            else {
                /* Aligned depth is off -> So, we are going to record the depth picture without the correction to match with the rgb picture */
                sDepthTopic = "/camera/depth/image_rect_raw";
            }
        }
        /* ROS master interaction to check wheter some topics are published or not */
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);
        ROS_INFO("Checking for available topic to subscribe...");
        for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
            const ros::master::TopicInfo& info = *it;
            /* Subscribe to the default topics for grabbing the camera pictures */
            if (info.name == sColorTopic) {
                sColorSubs = nh_.subscribe<sensor_msgs::Image>(sColorTopic,1,&image_conversion::color_image_callback,this);
                bColorOn = true;
                sOutMessage = "Connection with " + sColorTopic + " topic done...";
                ROS_INFO(sOutMessage.data());
                sOutMessage.clear();
            }
            if (info.name == sInfra1Topic) {
                sInfra1Subs = nh_.subscribe<sensor_msgs::Image>(sInfra1Topic,1,&image_conversion::infra1_image_callback,this);
                bInfra1On = true;
                sOutMessage = "Connection with " + sInfra1Topic + " topic done...";
                ROS_INFO(sOutMessage.data());
                sOutMessage.clear();
            }
            if (info.name == sInfra2Topic) {
                sInfra1Subs = nh_.subscribe<sensor_msgs::Image>(sInfra2Topic,1,&image_conversion::infra1_image_callback,this);
                bInfra2On = true;
                sOutMessage = "Connection with " + sInfra2Topic + " topic done...";
                ROS_INFO(sOutMessage.data());
                sOutMessage.clear();
            }
            if (info.name == sDepthTopic) {
                sDepthSubs = nh_.subscribe<sensor_msgs::Image>(sDepthTopic,1,&image_conversion::depth_image_callback,this);   
                bDepthOn = true;
                sOutMessage = "Connection with " + sDepthTopic + " topic done...";
                ROS_INFO(sOutMessage.data());
                sOutMessage.clear();
            }
            // std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << std::endl;
        }  
        /* Checking the topic connection that has been done */
        if (!bInfra1On) 
            ROS_WARN("Connection with /camera/infra1/image_rect_raw topic dropped...");
        if (!bInfra2On)
            ROS_WARN("Connection with /camera/infra2/image_rect_raw topic dropped...");
        if (!bColorOn) {
            ROS_ERROR("Connection with /camera/color/image_raw cannot be done... Please, check the connection and launch again the node...");
            exit(EXIT_FAILURE);
        }
        if (!bDepthOn) {
            ROS_ERROR("Connection with /camera/*_depth_*/image_raw cannot be done... Please, check the connection and launch again the node...");
            exit(EXIT_FAILURE);
        }
        /* Initialization value of non-ROS attributes of the class */
        bColorReceived = false;
        bInfra1Received = false;
        bInfra2Received = false;
        bDepthReceived = false;       
    }

    /***********************************************************
     * ROS CALLBACK MEHTODS OF THE CLASS
     **********************************************************/ 
    void image_conversion::color_image_callback(const sensor_msgs::Image::ConstPtr &imageMsgPtr) {
        try {
            imGrabbedColor = *imageMsgPtr;
            try {
                cv_bridge::CvImagePtr cvColorPtr = cv_bridge::toCvCopy(imageMsgPtr, sensor_msgs::image_encodings::BGR8);
                mColorImage = cvColorPtr->image;
            }
            catch (cv_bridge::Exception &eCV) {
                ROS_ERROR("Error occured in [cv_bridge]: %s", eCV.what());
                return;
            }
            bColorReceived = true;
        }
        catch (ros::Exception &e) {
            ROS_ERROR("Error occured in [color_image_callback]: %s", e.what());
            bColorReceived = false;
        }
    }

    void image_conversion::infra1_image_callback(const sensor_msgs::Image::ConstPtr &imageMsgPtr) {
        try {
            imGrabbedInfra1 = *imageMsgPtr;
            try {
                cv_bridge::CvImagePtr cvInfra1Ptr = cv_bridge::toCvCopy(imageMsgPtr, sensor_msgs::image_encodings::MONO8);
                 mInfra1Image = cvInfra1Ptr->image;
            }
            catch (cv_bridge::Exception &eCV) {
                ROS_ERROR("Error occured in [cv_bridge]: %s", eCV.what());
                return;
            }
            bInfra1Received = true;
        }
        catch (ros::Exception &e) {
            ROS_ERROR("Error occured in [infra1_image_callback]: %s", e.what());
            bInfra1Received = false;
        }
    }

    void image_conversion::infra2_image_callback(const sensor_msgs::Image::ConstPtr &imageMsgPtr) {
        try {
            imGrabbedInfra2 = *imageMsgPtr;
            try {
                cv_bridge::CvImagePtr cvInfra2Ptr = cv_bridge::toCvCopy(imageMsgPtr, sensor_msgs::image_encodings::MONO8);
                mInfra2Image = cvInfra2Ptr->image;
            }
            catch (cv_bridge::Exception &eCV) {
                ROS_ERROR("Error occured in [cv_bridge]: %s", eCV.what());
                return;
            }
            bInfra2Received = true;
        }
        catch (ros::Exception &e) {
            ROS_ERROR("Error occured in [infra2_image_callback]: %s", e.what());
            bInfra2Received = false;
        }
    }

    void image_conversion::depth_image_callback(const sensor_msgs::Image::ConstPtr &imageMsgPtr) {
        try {
            imGrabbedDepth = *imageMsgPtr;
            try {
                cv_bridge::CvImagePtr cvDepthPtr = cv_bridge::toCvCopy(imageMsgPtr, sensor_msgs::image_encodings::TYPE_16UC1);  // TYPE_16UC1  TYPE_8UC1    TYPE_32FC1
                mDepthImage = cvDepthPtr->image;
            }   
            catch (cv_bridge::Exception &eCV) {
                ROS_ERROR("Error occured in [cv_bridge]: %s", eCV.what());
                return;
            }
            bDepthReceived = true;
        }
        catch (ros::Exception &e) {
            ROS_ERROR("Error occured in [depth_image_callback]: %s", e.what());
            bDepthReceived = false;
        }
    }

    /***********************************************************
     * CONSTRUCTOR AND DESTRUCTOR OF THE CLASS
     **********************************************************/ 
    image_conversion::image_conversion(std::string sCameraNumber, bool bAlignedDepthOn) {
        _bAlignedDepth = true;
        init(sCameraNumber, bAlignedDepthOn);
    }

    image_conversion::~image_conversion() {     // TODO: Fix what happens when the app is closed without instantiation of a Graphic window
        if(!cv::getWindowProperty(sOpencvWindow,cv::WND_PROP_AUTOSIZE) == -1 ){
            cv::destroyAllWindows();
        }
    }

    /***********************************************************
     *  PUBLIC METHODS OF THE CLASS
     **********************************************************/ 
    bool image_conversion::displayImage(short int siNumImages) {
        if (siNumImages > 0 || siNumImages <= 4){
            cv::namedWindow(sOpencvWindow, cv::WINDOW_AUTOSIZE);
            /* Update GUI window */
            switch (siNumImages) {
                case 1:  // Picturing just the color camera
                    cv::imshow(sOpencvWindow, mColorImage);  
                    break;
                case 2:  // Picturing just the infra1 camera
                    if (bInfra1On) {
                        cv::imshow(sOpencvWindow, mInfra1Image);
                    }
                    else {
                        ROS_WARN("ROS Node is not connected to infra1 camera topics");
                        return false;
                    }
                    break;
                case 3:  // Picturing just the infra 2 camera
                    if (bInfra2On) {
                        cv::imshow(sOpencvWindow, mInfra2Image);
                    }
                    else {
                        ROS_WARN  ("ROS Node is not connected to infra2 camera topics");
                        return false;
                    }                    
                    break;
                default:    // Picturing just the depth camera
                    cv::Mat mDepthImageAux = mDepthImage.clone();
                    cv::convertScaleAbs(mDepthImageAux, mDepthImageAux, 0.03);                                                      // Changing the color scale of the readed Disparity Depth Map
                    cv::applyColorMap(mDepthImageAux,mDepthImageAux, cv::COLORMAP_JET);                                                   // Setting the colour of the mDepthImage
                    cv::imshow(sOpencvWindow, mDepthImageAux);
            }
            cv::waitKey(5);
            return true;
        }
        else {
            ROS_INFO("The number is out of the ranges [1-4]");
            return false;
        }
    }

    bool image_conversion::computeDisparityMapFromStereo() {
        if (bInfra1On && bInfra2On) {
            try {
                cv::Mat mLeftImage;
                cv::Mat mRightImage;

                cv::Size size(640,480);
                cv::resize(mInfra1Image, mRightImage, size);
                cv::resize(mInfra2Image, mLeftImage, size);
            
                int iNumDisparities = 16, iBlockSize = 15;
                cv::Ptr<cv::StereoBM> ptrStereoImage = cv::StereoBM::create(iNumDisparities, iBlockSize);

                ptrStereoImage->compute(mInfra1Image, mInfra2Image, mComputedDisparityMap);

                cv::namedWindow(sOpencvWindow, cv::WINDOW_NORMAL);
                cv::imshow(sOpencvWindow, mComputedDisparityMap);  
                cv::waitKey(5);
            }
            catch (cv::Exception &e) {
                ROS_ERROR("Error ocurred while computing disparity map: %s", e.what());
                return false;
            }
            return true;
        }
        else {
            ROS_ERROR("Cannot compute disparity map due to not receiving infra pictures");
            return false;
        }
    }

    /***********************************************************
     * SETTERS AND GETTERS OF THE CLASS
     **********************************************************/ 
    sensor_msgs::Image image_conversion::getimGrabbedColor() {
        return imGrabbedColor;
    }

    sensor_msgs::Image image_conversion::getimGrabbedInfra1() {
        return imGrabbedInfra1;
    }

    sensor_msgs::Image image_conversion::getimGrabbedInfra2() {
        return imGrabbedInfra2;
    }

    sensor_msgs::Image image_conversion::getimGrabbedDepth() {
        return imGrabbedDepth;
    }

    cv::Mat image_conversion::getmColorImage() {
        return mColorImage;
    }

    cv::Mat image_conversion::getmInfra1Image() {
        return mInfra1Image;
    }

    cv::Mat image_conversion::getmInfra2Image() {
        return mInfra2Image;
    }
    
    cv::Mat image_conversion::getmDepthImage() {
        return mDepthImage;
    }


    bool image_conversion::getbColorReceived() {
        return bColorReceived;
    }

    bool image_conversion::getbInfra1Received() {
        return bInfra1Received;
    }

    bool image_conversion::getbInfra2Received() {
        return bInfra2Received;
    }

    bool image_conversion::getbDepthReceived() {
        return bDepthReceived;
    }

    bool image_conversion::getbColorOn() {
        return bColorOn;
    }

    bool image_conversion::getbInfra1On() {
        return bInfra1On;
    }

    bool image_conversion::getbInfra2On() {
        return bInfra2On;
    }

    bool image_conversion::getbDepthOn() {
        return bDepthOn;
    }

    bool image_conversion::isAlignedDepthOn() {
        return _bAlignedDepth;
    }

    void image_conversion::clearbColorReceived() {
        bColorReceived = false;
    }

    void image_conversion::clearbInfra1Received() {
        bInfra1Received = false;
    }

    void image_conversion::clearbInfra2Received() {
        bInfra2Received = false;
    }

    void image_conversion::clearbDepthReceived() {
        bDepthReceived = false;
    }

    /***********************************************************
     * PROTECTED METHODS OF THE CLASS
     **********************************************************/ 

}