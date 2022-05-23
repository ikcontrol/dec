#include "../../include/vision/image_processing_class.h"

namespace pic_handling
{
    /***********************************************************
     * PRIVATE METHODS OF THE CLASS
     **********************************************************/ 
    void image_processing::main_loop() {
        /* Non-cyclic part of the main loop. For example, it can be used for moving the robot to home position (it is not this particular case) */
        double dTime = (double)cv::getTickCount();
      
       
        /* Cyclic part of the main loop */
        while (ros::ok()) {
            /* First read and store cyclicly the images */
            if(icCamGrabber1.getbColorReceived() && !bIsProcessing){
                icCamGrabber1.getmColorImage().copyTo(mColorPic1);
                bColorStored1 = true;
                icCamGrabber1.clearbColorReceived();
            }
            if(icCamGrabber2.getbColorReceived() && !bIsProcessing) {
                icCamGrabber2.getmColorImage().copyTo(mColorPic2);
                bColorStored2 = true;
                icCamGrabber2.clearbColorReceived();
            }
            if(icCamGrabber1.getbDepthReceived() && !bIsProcessing) {
                icCamGrabber1.getmDepthImage().copyTo(mDepthPic1);
                // icCamGrabber1.getmDepthImage().copyTo(mDepthDistance1);
                // mDepthDistance1 = mDepthDistance1 * 0.001;
                icCamGrabber1.getmDepthImage().copyTo(mDepthColor1);
                // cv::minMaxLoc(mDepthColor1, &minValue, &maxValue);
                cv::convertScaleAbs(mDepthColor1, mDepthColor1, 0.03);
                cv::applyColorMap(mDepthColor1, mDepthColor1, cv::COLORMAP_JET);
                bDepthStored1 = true;
                icCamGrabber1.clearbDepthReceived();
            }
            if(icCamGrabber2.getbDepthReceived() && !bIsProcessing){
                icCamGrabber2.getmDepthImage().copyTo(mDepthPic2);
                // icCamGrabber2.getmDepthImage().copyTo(mDepthDistance2);
                // mDepthDistance2 = mDepthDistance2 * 0.001;
                icCamGrabber2.getmDepthImage().copyTo(mDepthColor2);
                // cv::minMaxLoc(mDepthColor2, &minValue, &maxValue);
                cv::convertScaleAbs(mDepthColor2, mDepthColor2, 0.03);
                cv::applyColorMap(mDepthColor2, mDepthColor2, cv::COLORMAP_JET);
                bDepthStored2 = true;
                icCamGrabber2.clearbDepthReceived();
            }
            if(bColorStored1 && bColorStored2 && bDepthStored1 && bDepthStored2 && !bIsProcessing) {
                bIsProcessing = true;
                bColorStored1 = false;
                bColorStored2 = false;
                bDepthStored1 = false;
                bDepthStored2 = false;
            }

            /* Processing the stored images for obtaining the detected obstacles positions */
            if(bIsProcessing){
                dTime = (double)cv::getTickCount();

                /* Processing each of the pictures for obtaining positioning box */
                // TODO: Check if there's need for parallel computing for every of the pictures
                odZYCam1 = process_obstacles_zy_plane();                                   // Processing the number of obstacles of the camera1 picture
                odXYCam2 = process_obstacles_xy_plane();                                   // Processing the number of obstacles of the camera2 picture


                /* Publishing depth into depth format image for ROS */
                // sensor_msgs::CameraInfo ciMsg;   
                // sensor_msgs::Image img_msg;
                // cv_bridge::CvImage cv_img_bridge;
                // std_msgs::Header hHeader;
                // ciMsg = getCam1DepthInfo();
                // ciMsg.header.frame_id = "rs_d435_cam_color_optical_frame";
                // ciMsg.header.stamp = ros::Time::now();
                // pcl_info_pub.publish(ciMsg);
                // hHeader.stamp = ros::Time::now();
                // hHeader.frame_id = "rs_d435_cam_color_optical_frame";
                // cv_img_bridge = cv_bridge::CvImage(hHeader, sensor_msgs::image_encodings::TYPE_16UC1, mDepthDistance1);
                // cv_img_bridge.toImageMsg(img_msg);
                // pcl_img_pub.publish(img_msg);
                // cv::imshow("Depth Test Image", mDepthDistance1);
                // ROS_INFO("Exited the image color filter");

                


                /* Combination of the information of each positioning box for obstacles representation and occlusion avoidance */
                // ocpXYZPositions = compute_xyz_obstacle_position();      // Processing the carthesian position for each of the obstacles

                /* Publishing the information into the ROS net to be used by other nodes or the orocos net to support the control architecture */
                publish_obs_point_cloud();

                /* Releasing bIsProcessing attribute for a new cycle */
                bIsProcessing = false;
                
                /* Computing the processing time for the whole algorithms */
                dTime = ((double)cv::getTickCount() - dTime)/cv::getTickFrequency();
                ROS_INFO("El tiempo elapsed ha sido: %f seconds\n", dTime);
                ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
                // cv::waitKey();
                // cv::destroyAllWindows();
            }
        }
    }

    /* Processing the obstacles of the ZY plane method -> process_obstacles_zy_plane() */
    obstacle_data image_processing::process_obstacles_zy_plane() {
        // TODO: Check performance with cv::Mat in this method or as a stored variables in the instantation
        cv::Mat mColorFiltered, mHsvFiltered, mGrayFiltered, mSceneGreenMask, mSceneGreenColor, mSceneGreenGray;             // Instantiation of the variables for cmaera 1 pictures
        cv::Mat mRectanglesPic = cv::Mat::zeros(cv::Size(ciColorInfo1.width, ciColorInfo1.height), CV_8UC3);
        cv::Mat mRectanglesPicGray = cv::Mat::zeros(cv::Size(ciColorInfo1.width, ciColorInfo1.height), CV_8UC1);
        cv::Mat mContourMask = cv::Mat::zeros(cv::Size(ciColorInfo1.width, ciColorInfo1.height), CV_8UC3);
        cv::Mat mContourMaskGray = cv::Mat::zeros(cv::Size(ciColorInfo1.width, ciColorInfo1.height), CV_8UC1);
        cv::Mat mContourMaskROI = cv::Mat::zeros(cv::Size(ciColorInfo1.width, ciColorInfo1.height), CV_8UC1);
        mDepthDistance1 = cv::Mat::zeros(cv::Size(ciColorInfo1.width, ciColorInfo1.height), CV_16UC1);
                
        std::vector<std::vector<cv::Point>> vContours;
        std::vector<std::vector<cv::Point>> vContoursRect;
        std::vector<std::vector<cv::Point>> vContoursReduced;
        std::vector<cv::Vec4i> vHierarchy;
        std::vector<cv::Vec4i> vHierarchyRect;
        std::vector<cv::Vec4i> vHierarchyReduced;
        
        std::vector<int> vDetectedObstaclesIndex;
        std::vector<double> vDetectedObstaclesAreas;
        
        obstacle_data odVar;

        int iDetectedObstacles = 0;
        double dAuxArea = 0.0;
        double dFurthestPoint = 0.0, dClosestPoint = 0.0;
        cv::Point pClosest(0, 0), pFurthest(0, 0);

        /* Working with camera 1 pictures (for both, color and depth) */
        /* Preparing the filtered picture, the hsv picture and the grayscale picture to work with them */
        cv::GaussianBlur(mColorPic1, mColorFiltered, cv::Size(3,3), 0);
        cv::cvtColor(mColorFiltered,mHsvFiltered,cv::COLOR_BGR2HSV);
        cv::cvtColor(mColorFiltered, mGrayFiltered, cv::COLOR_BGR2GRAY);

        /* Extracting the green obstacles by using a mask */
        cv::inRange(mHsvFiltered, sLowerThreshold, sUpperThreshold, mSceneGreenMask);
        cv::bitwise_and(mGrayFiltered, mSceneGreenMask, mSceneGreenGray);
        cv::cvtColor(mSceneGreenGray, mSceneGreenColor, cv::COLOR_GRAY2BGR);

        /* Once the green obstacles has been segmented from the picture, it must be counted through their contours */
        cv::findContours(mSceneGreenGray, vContours, vHierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        for(size_t i = 0; i < vContours.size(); i++) {
            dAuxArea = cv::contourArea(vContours[i]);
            if(dAuxArea > dAreaThreshold) {
                vDetectedObstaclesIndex.push_back(i);
                vDetectedObstaclesAreas.push_back(dAuxArea);
                iDetectedObstacles++;
            }
        }

        /* Obtaining the data from the obstacles through 4 steps */
        if(iDetectedObstacles != 0) {
            cv::Rect rMinRectangle;
            mContourMask = cv::Mat::zeros(cv::Size(ciColorInfo1.width, ciColorInfo1.height), CV_8UC3);
            mContourMaskGray = cv::Mat::zeros(cv::Size(ciColorInfo1.width, ciColorInfo1.height), CV_8UC1);
            std::vector<std::vector<int>> box;                                                                                  // z_min, z_max, y_min, y_max for each object rectangle
            std::vector<int> auxBox;                                                                                            // z_min, z_max, y_min, y_max for each object rectangle
            std::vector<double> vAuxPoses;                                                                                      // z1, y1, z2, y2, z3, y3, z4, y4
            obstacle_parameters opParams;                                                                                       // (closest_dist, furthest_dist, y_center, z_center, y_low, y_high, z_low, z_high)             
            
            for(size_t i = 0; i < iDetectedObstacles; i++) {
                auxBox.clear();
                cv::Scalar sColor(rand()%255, rand()%255, rand()%255);
                cv::drawContours(mSceneGreenColor, vContours, vDetectedObstaclesIndex[i], cv::Scalar(0,0,255), 2);               // drawing the rectangle in bgr pic

                /* Step 1) Find the closest point to the orthonormal plane to the focal line */
                cv::drawContours(mContourMask, vContours, vDetectedObstaclesIndex[i], sColor, cv::FILLED);
                cv::cvtColor(mContourMask, mContourMaskGray, cv::COLOR_BGR2GRAY);
                cv::threshold(mContourMaskGray, mContourMaskROI, 25, 255, cv::THRESH_BINARY);
                cv::erode(mContourMaskROI, mContourMaskROI, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(iKernelSize,iKernelSize)));
                cv::bitwise_and(mDepthPic1, mDepthPic1, mDepthDistance1, mContourMaskROI);                               
                mDepthDistance1.setTo(10200, mDepthDistance1 < 20);


                /* Step 2) Finding the smallest rectangle which fits in the contour */
                rMinRectangle = cv::boundingRect(vContours[vDetectedObstaclesIndex[i]]);
                cv::rectangle(mRectanglesPic, rMinRectangle, sColor, cv::FILLED);                                               // TODO: comment this line, it is used to represent in bgr pictures the rectancles of detected obstacles
                cv::cvtColor(mRectanglesPic, mRectanglesPicGray, cv::COLOR_BGR2GRAY);                                           // TODO: comment this line
                cv::findContours(mRectanglesPicGray, vContoursRect, vHierarchyRect, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);    // TODO: comment this line
                cv::drawContours(mColorFiltered, vContoursRect, -1, cv::Scalar(255, 255, 0), 2);                                // TODO: comment this line after debugging
                cv::drawContours(mDepthColor1, vContoursRect, -1, cv::Scalar(255, 255, 0), 2);                                  // TODO: comment this line after debugging
                cv::drawContours(mRectanglesPic, vContours, vDetectedObstaclesIndex[i], cv::Scalar(0, 255, 0), 2);              // TODO: check if commenting this line
                cv::findContours(mContourMaskROI, vContoursReduced, vHierarchyReduced, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE); // TODO: check if commenting this line
                cv::drawContours(mRectanglesPic, vContoursReduced, -1, cv::Scalar(150, 150, 0), 2);                             // TODO: check if commenting this line
                cv::drawContours(mColorFiltered, vContoursReduced, -1, cv::Scalar(0, 0, 255), 2);                               // TODO: check if commenting this line
                cv::drawContours(mDepthColor1, vContoursReduced, -1, cv::Scalar(150, 150, 0), 2);                               // TODO: check if commenting this line
                // auxBox.push_back(rMinRectangle.x);                                                                              // x_pixel_min -> y_world_min -> [0]
                // auxBox.push_back(rMinRectangle.x + rMinRectangle.width);                                                        // x_pixel_max -> y_world_max -> [1]
                // auxBox.push_back(rMinRectangle.y);                                                                              // y_pixel_min -> z_world_min -> [2]
                // auxBox.push_back(rMinRectangle.y + rMinRectangle.height);                                                       // y_pixel_max -> z_world_max -> [3]
                // box.push_back(auxBox);
                // // ROS_INFO("Coordinates of the box1.%d in pixels: y_min=%d, y_max=%d, z_min=%d, z_max=%d", i, box[i].data()[0], box[i].data()[1], box[i].data()[2], box[i].data()[3]);

                // /* Step 3) Computation of (y, z) positions for the proyection to the furthest plane */
                // vAuxPoses.clear();
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[2] - dCam1_cy) / dCam1_fy);                                        // z1 = z_min -> 0
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[0] - dCam1_cx) / dCam1_fx);                                        // y1 = y_min -> 1
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[3] - dCam1_cy) / dCam1_fy);                                        // z2 = z_max -> 2
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[0] - dCam1_cx) / dCam1_fx);                                        // y2 = y_min -> 3
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[3] - dCam1_cy) / dCam1_fy);                                        // z3 = z_max -> 4 
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[1] - dCam1_cx) / dCam1_fx);                                        // y3 = y_max -> 5
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[2] - dCam1_cy) / dCam1_fy);                                        // z4 = z_min -> 6
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[1] - dCam1_cx) / dCam1_fx);                                        // y4 = y_max -> 7
                // // ROS_INFO("Furthest plane positions: x_min=%f, x_max=%f, y_min=%f, y_max=%f, z_min=%f, z_max=%f", dClosestPoint, dFurthestPoint, vAuxPoses[1], vAuxPoses[5], vAuxPoses[0], vAuxPoses[2]);

                // /* Step 4) Extracting the charactization of the real world containing box */
                // // vAuxParams.clear();
                // opParams.dClosestPoint = dClosestPoint;                                                                         // Closest depth (x_closest)    -> 0
                // opParams.dFurthestPoint = dFurthestPoint;                                                                       // Furthest_depth (x_furthest)  -> 1
                // opParams.dWidthCenter = vAuxPoses[1] + (vAuxPoses[5] - vAuxPoses[1]) / 2;                                       // Y center                     -> 2
                // opParams.dHeightCenter = vAuxPoses[0] + (vAuxPoses[2] - vAuxPoses[0]) / 2;                                      // Z center                     -> 3
                // opParams.dWidthLower = vAuxPoses[1];                                                                            // Y_min                        -> 4
                // opParams.dWidthHigher = vAuxPoses[5];                                                                           // Y_max                        -> 5
                // opParams.dHeightLower = vAuxPoses[0];                                                                           // Z_min                        -> 6
                // opParams.dHeightHigher = vAuxPoses[2];                                                                          // Z_max                        -> 7
                // // ROS_INFO("Camera 1: x_closest=%f, x_furthest=%f, Y_center=%f, Z_center=%f", opParams.dClosestPoint, opParams.dFurthestPoint, opParams.dWidthCenter, opParams.dHeightCenter);
                // odVar.vopObsParam.push_back(opParams);
                // // printf("The cube paremeters:\n");
                // // printf("  Closest X plane: %f\n", odVar.vopObsParam[i].dClosestPoint);
                // // printf("  Furthest X plane: %f\n", odVar.vopObsParam[i].dFurthestPoint);
                // // printf("  Center (Y,Z): (%f,%f)\n", odVar.vopObsParam[i].dWidthCenter, odVar.vopObsParam[i].dHeightCenter);
                // // printf("  First XZ plane (lowest Y): %f\n", odVar.vopObsParam[i].dWidthLower);
                // // printf("  Second XZ plane (highest Y): %f\n", odVar.vopObsParam[i].dWidthHigher);
                // // printf("  First XY plane (lowest Z): %f\n", odVar.vopObsParam[i].dHeightLower);
                // // printf("  Second XY plane (highest Z): %f\n", odVar.vopObsParam[i].dHeightHigher);
            }
            /* Storing the total amount of obstacles found */
            odVar.siObsFound = iDetectedObstacles;
            mColorFiltered.copyTo(mColorProcessed1);

            /* Picturing the taken pictures and if there's any obstacle recogniced */ 
            cv::imshow("Camera 1 - RGB Picture", mColorProcessed1);              // mColorProcessed1  mSceneGreenColor       mContourMaskROI   mContourMask   mSceneGreenColor  mContourMaskROI    mContourMaskDistance
            cv::waitKey(10);
        }
        else {
            odVar.siObsFound = 0;
            cv::imshow("Camera 1 - RGB Picture", mColorPic1);
            // ROS_INFO("No obstacles has been detected");
        }

        return odVar;
    }
    

    /* Processing the obstacles of the XY plane -> process_obstacles_xy_plane() */
    obstacle_data image_processing::process_obstacles_xy_plane() {
        // TODO: Check performance with cv::Mat in this method or as a stored variables in the instantation
        cv::Mat mColorFiltered, mHsvFiltered, mGrayFiltered, mSceneGreenMask, mSceneGreenColor, mSceneGreenGray;             // Instantiation of the variables for cmaera 1 pictures
        cv::Mat mRectanglesPic = cv::Mat::zeros(cv::Size(ciColorInfo2.width, ciColorInfo2.height), CV_8UC3);
        cv::Mat mRectanglesPicGray = cv::Mat::zeros(cv::Size(ciColorInfo2.width, ciColorInfo2.height), CV_8UC1);
        cv::Mat mContourMask = cv::Mat::zeros(cv::Size(ciColorInfo2.width, ciColorInfo2.height), CV_8UC3);
        cv::Mat mContourMaskGray = cv::Mat::zeros(cv::Size(ciColorInfo2.width, ciColorInfo2.height), CV_8UC1);
        cv::Mat mContourMaskROI = cv::Mat::zeros(cv::Size(ciColorInfo2.width, ciColorInfo2.height), CV_8UC1);
        // cv::Mat mContourMaskDistance = cv::Mat::zeros(cv::Size(ciColorInfo1.width, ciColorInfo1.height), CV_16UC1);
        mDepthDistance2 = cv::Mat::zeros(cv::Size(ciColorInfo1.width, ciColorInfo1.height), CV_16UC1);
                
        std::vector<std::vector<cv::Point>> vContours;
        std::vector<std::vector<cv::Point>> vContoursRect;
        std::vector<std::vector<cv::Point>> vContoursReduced;
        std::vector<cv::Vec4i> vHierarchy;
        std::vector<cv::Vec4i> vHierarchyRect;
        std::vector<cv::Vec4i> vHierarchyReduced;
        
        std::vector<int> vDetectedObstaclesIndex;
        std::vector<double> vDetectedObstaclesAreas;
        
        obstacle_data odVar;

        int iDetectedObstacles = 0;
        double dAuxArea = 0.0;
        double dFurthestPoint = 0.0, dClosestPoint = 0.0;

        /* Working with camera 1 pictures (for both, color and depth) */
        /* Preparing the filtered picture, the hsv picture and the grayscale picture to work with them */
        cv::GaussianBlur(mColorPic2, mColorFiltered, cv::Size(3,3), 0);
        cv::cvtColor(mColorFiltered,mHsvFiltered,cv::COLOR_BGR2HSV);
        cv::cvtColor(mColorFiltered, mGrayFiltered, cv::COLOR_BGR2GRAY);

        /* Extracting the green obstacles by using a mask */
        cv::inRange(mHsvFiltered, sLowerThreshold, sUpperThreshold, mSceneGreenMask);
        cv::bitwise_and(mGrayFiltered, mSceneGreenMask, mSceneGreenGray);
        cv::cvtColor(mSceneGreenGray, mSceneGreenColor, cv::COLOR_GRAY2BGR);

        /* Once the green obstacles has been segmented from the picture, it must be counted through their contours */
        cv::findContours(mSceneGreenGray, vContours, vHierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        for(size_t i = 0; i < vContours.size(); i++) {
            dAuxArea = cv::contourArea(vContours[i]);
            if(dAuxArea > dAreaThreshold) {
                vDetectedObstaclesIndex.push_back(i);
                vDetectedObstaclesAreas.push_back(dAuxArea);
                iDetectedObstacles++;
            }
        }

        /* Obtaining the data from the obstacles through 4 steps */
        if(iDetectedObstacles != 0) {
            cv::Rect rMinRectangle;
            mContourMask = cv::Mat::zeros(cv::Size(ciColorInfo2.width, ciColorInfo2.height), CV_8UC3);
            mContourMaskGray = cv::Mat::zeros(cv::Size(ciColorInfo2.width, ciColorInfo2.height), CV_8UC1);
            std::vector<std::vector<int>> box;                                                                                  // x_min, x_max, y_min, y_max for each object rectangle
            std::vector<int> auxBox;                                                                                            // x_min, x_max, y_min, y_max for each object rectangle
            std::vector<double> vAuxPoses;                                                                                      // x1, y1, x2, y2, x3, y3, x4, y4
            obstacle_parameters opParams;                                                                                       // (closest_dist, furthest_dist, y_center, x_center, y_low, y_high, x_low, x_high)             
            
            for(size_t i = 0; i < iDetectedObstacles; i++) {
                auxBox.clear();
                cv::Scalar sColor(rand()%255, rand()%255, rand()%255);
                cv::drawContours(mSceneGreenColor, vContours, vDetectedObstaclesIndex[i], cv::Scalar(0,0,255), 2);               // drawing the rectangle in bgr pic
                // cv::drawContours(mSceneDepth, vContours, i, cv::Scalar(0,0,255), 2);                                             // scene_depth_bgr_pìc in python algorithm

                /* Step 1) Find the closest point to the orthonormal plane to the focal line */
                cv::drawContours(mContourMask, vContours, vDetectedObstaclesIndex[i], sColor, cv::FILLED);
                cv::cvtColor(mContourMask, mContourMaskGray, cv::COLOR_BGR2GRAY);
                cv::threshold(mContourMaskGray, mContourMaskROI, 25, 255, cv::THRESH_BINARY);
                cv::erode(mContourMaskROI, mContourMaskROI, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(iKernelSize,iKernelSize)));
                cv::bitwise_and(mDepthPic2, mDepthPic2, mDepthDistance2, mContourMaskROI);                                 // mDepthDistance1    
                mDepthDistance2.setTo(10200, mDepthDistance2 < 20);


                /* Step 2) Finding the smallest rectangle which fits in the contour */
                rMinRectangle = cv::boundingRect(vContours[vDetectedObstaclesIndex[i]]);
                cv::rectangle(mRectanglesPic, rMinRectangle, sColor, cv::FILLED);                                               // TODO: comment this line, it is used to represent in bgr pictures the rectancles of detected obstacles
                cv::cvtColor(mRectanglesPic, mRectanglesPicGray, cv::COLOR_BGR2GRAY);                                           // TODO: comment this line
                cv::findContours(mRectanglesPicGray, vContoursRect, vHierarchyRect, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);    // TODO: comment this line
                cv::drawContours(mColorFiltered, vContoursRect, -1, cv::Scalar(255, 255, 0), 2);                                // TODO: comment this line after debugging
                cv::drawContours(mDepthColor2, vContoursRect, -1, cv::Scalar(255, 255, 0), 2);                                  // TODO: comment this line after debugging
                cv::drawContours(mRectanglesPic, vContours, vDetectedObstaclesIndex[i], cv::Scalar(0, 255, 0), 2);              // TODO: check if commenting this line
                cv::findContours(mContourMaskROI, vContoursReduced, vHierarchyReduced, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE); // TODO: check if commenting this line
                cv::drawContours(mRectanglesPic, vContoursReduced, -1, cv::Scalar(150, 150, 0), 2);                             // TODO: check if commenting this line
                cv::drawContours(mColorFiltered, vContoursReduced, -1, cv::Scalar(0, 0, 255), 2);                               // TODO: check if commenting this line
                cv::drawContours(mDepthColor1, vContoursReduced, -1, cv::Scalar(150, 150, 0), 2);                               // TODO: check if commenting this line
                // auxBox.push_back(rMinRectangle.x);                                                                              // x_pixel_min -> y_world_min -> [0]
                // auxBox.push_back(rMinRectangle.x + rMinRectangle.width);                                                        // x_pixel_max -> y_world_max -> [1]
                // auxBox.push_back(rMinRectangle.y);                                                                              // y_pixel_min -> x_world_min -> [2]
                // auxBox.push_back(rMinRectangle.y + rMinRectangle.height);                                                       // y_pixel_max -> x_world_max -> [3]
                // box.push_back(auxBox);
                // // ROS_INFO("Coordinates of the box1.%d in pixels: x_min=%d, x_max=%d, y_min=%d, y_max=%d", i, box[i].data()[2], box[i].data()[3], box[i].data()[0], box[i].data()[1]);

                // /* Step 3) Computation of (y, z) positions for the proyection to the furthest plane */
                // vAuxPoses.clear();
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[2] - dCam1_cy) / dCam1_fy);                                        // x1 = x_min -> 0
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[0] - dCam1_cx) / dCam1_fx);                                        // y1 = y_min -> 1
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[3] - dCam1_cy) / dCam1_fy);                                        // x2 = x_max -> 2
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[0] - dCam1_cx) / dCam1_fx);                                        // y2 = y_min -> 3
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[3] - dCam1_cy) / dCam1_fy);                                        // x3 = x_max -> 4 
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[1] - dCam1_cx) / dCam1_fx);                                        // y3 = y_max -> 5
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[2] - dCam1_cy) / dCam1_fy);                                        // x4 = x_min -> 6
                // vAuxPoses.push_back(dFurthestPoint * (auxBox[1] - dCam1_cx) / dCam1_fx);                                        // y4 = y_max -> 7
                // // ROS_INFO("Furthest plane positions: x_min=%f, x_max=%f, y_min=%f, y_max=%f, z_min=%f, z_max=%f", vAuxPoses[0], vAuxPoses[2], vAuxPoses[1], vAuxPoses[5], dClosestPoint, dFurthestPoint);

                // /* Step 4) Extracting the charactization of the real world containing box */
                // // vAuxParams.clear();
                // opParams.dClosestPoint = dClosestPoint;                                                                         // Closest depth (z_closest)    -> 0
                // opParams.dFurthestPoint = dFurthestPoint;                                                                       // Furthest_depth (z_furthest)  -> 1
                // opParams.dWidthCenter = vAuxPoses[1] + (vAuxPoses[5] - vAuxPoses[1]) / 2;                                       // Y center                     -> 2
                // opParams.dHeightCenter = vAuxPoses[0] + (vAuxPoses[2] - vAuxPoses[0]) / 2;                                      // X center                     -> 3
                // opParams.dWidthLower = vAuxPoses[1];                                                                            // Y_min                        -> 4
                // opParams.dWidthHigher = vAuxPoses[5];                                                                           // Y_max                        -> 5
                // opParams.dHeightLower = vAuxPoses[0];                                                                           // X_min                        -> 6
                // opParams.dHeightHigher = vAuxPoses[2];                                                                          // X_max                        -> 7
                // // ROS_INFO("Camera 2: x_closest=%d, x_furthest=%d, Y_center=%d, Z_center=%d", opParams.dClosestPoint, opParams.dFurthestPoint, opParams.dWidthCenter, opParams.dHeightCenter);
                // odVar.vopObsParam.push_back(opParams);
                // // printf("The cube paremeters:\n");
                // // printf("  Closest Z plane: %f\n", odVar.vopObsParam[i].dClosestPoint);
                // // printf("  Furthest Z plane: %f\n", odVar.vopObsParam[i].dFurthestPoint);
                // // printf("  Center (X,Y): (%f,%f)\n", odVar.vopObsParam[i].dHeightCenter, odVar.vopObsParam[i].dWidthCenter);
                // // printf("  First YZ plane (lowest X): %f\n", odVar.vopObsParam[i].dHeightLower);
                // // printf("  Second YZ plane (highest X): %f\n", odVar.vopObsParam[i].dHeightHigher);
                // // printf("  First XZ plane (lowest Y): %f\n", odVar.vopObsParam[i].dWidthLower);
                // // printf("  Second XZ plane (highest Y): %f\n", odVar.vopObsParam[i].dWidthHigher);
            }
            /* Storing the total amount of obstacles found */
            odVar.siObsFound = iDetectedObstacles;
            mColorFiltered.copyTo(mColorProcessed2);    // mColorProcessed2

            /* Picturing the taken pictures and if there's any obstacle recogniced */ 
            cv::imshow("Camera 2 - RGB Picture", mColorProcessed2);              // mColorProcessed2  mSceneGreenColor       mContourMaskROI   mContourMask  mContourMaskROI      mContourMaskDistance
            cv::waitKey(10);
        }
        else {
            odVar.siObsFound = 0;
            cv::imshow("Camera 2 - RGB Picture", mColorPic2);
            ROS_INFO("No obstacles has been detected");
        }

        
        return odVar;
    }

    /* Method for computing xyz obstacle position as well as the minimum radius required */
    obs_carthesian_position image_processing::compute_xyz_obstacle_position() {
        obs_carthesian_position ocpVar;

        /* Processing algorithm implementation */
        if(odZYCam1.siObsFound != 0 || odXYCam2.siObsFound != 0){
            if (odZYCam1.siObsFound == odXYCam2.siObsFound) {
                /* Case where the same obstacles has been found in picture 1 and 2. It mean that it might happen occlusion while detecting but there's no case of hard occlusion (when one obstacle is divided or totally ocluded in one view) */
                // ROS_INFO("The program has entered in regular xyz computation");
                ocpVar = regular_xyz_computation();
            }
            else {
                /* Case where hard occlusion has happend. One obstacle is completely ocludded from one view or splitted into several parts */
                // ROS_INFO("The program has entered in hard occlusion xyz computation");
                ocpVar = hard_occlusion_xyz_computation();
            }
        }
        else {
            ocpVar.siNumberObstacles = 0;
            ocpVar.occObstaclePoses.clear();
            ocpVar.vsiPublishCamera.clear();
            // printf ("No obstacles has been found in any of the two pictures");
        }

        return ocpVar;
    }

    /* regular_xyz_computation() method for most of the situations of detected obstacles */
    obs_carthesian_position image_processing::regular_xyz_computation() {
        obs_carthesian_position ocpVar;
        obs_cartheseian_components occAux;

        std::vector<std::pair<int, int>> vCoupledIndex;
        std::vector<int> vMissmatchedIndex;

        bool bSameCenterY, bSameLowY, bSameHighY;
        int iMissmatchedResults;
        int iAux = 0;

        double dCam1XDiff = 0.0;
        double dCam2XDiff = 0.0;
        double dCam1YDiff = 0.0;
        double dCam2YDiff = 0.0;
        double dCam1ZDiff = 0.0;
        double dCam2ZDiff = 0.0;
        double dXdiff, dYdiff, dZdiff;

        /* Step 1) Checking the matched and miss-matched obstacles. As the found obstacles are the same in both situations, it doesn't matter the order */
        for (size_t i = 0; i < odZYCam1.siObsFound; i++) {
            iMissmatchedResults = 0;
            for (size_t j = 0; j < odXYCam2.siObsFound; j++) {
                bSameCenterY = false;
                bSameLowY = false;
                bSameHighY = false;
                if (odZYCam1.vopObsParam[i].dWidthCenter >= odXYCam2.vopObsParam[j].dWidthCenter - dDistanceThreshold && odZYCam1.vopObsParam[i].dWidthCenter <= odXYCam2.vopObsParam[j].dWidthCenter + dDistanceThreshold) {
                    bSameCenterY = true;
                }
                if (odZYCam1.vopObsParam[i].dWidthLower >= odXYCam2.vopObsParam[j].dWidthLower - dDistanceThreshold && odZYCam1.vopObsParam[i].dWidthLower <= odXYCam2.vopObsParam[j].dWidthLower + dDistanceThreshold) {
                    bSameLowY = true;
                }
                if (odZYCam1.vopObsParam[i].dWidthHigher >= odXYCam2.vopObsParam[j].dWidthHigher - dDistanceThreshold && odZYCam1.vopObsParam[i].dWidthHigher <= odXYCam2.vopObsParam[j].dWidthHigher + dDistanceThreshold) {
                    bSameHighY = true;
                }
                if (bSameCenterY || bSameLowY || bSameHighY) {
                    // ROS_INFO("    Obstacle matched");
                    vCoupledIndex.push_back(std::make_pair(i, j));
                }
                else {
                    // ROS_INFO("    Obstacle miss-matched");
                    iMissmatchedResults++;
                }
            }
            if (iMissmatchedResults == odXYCam2.siObsFound) {
                // ROS_INFO("There's a missmatched element");
                vMissmatchedIndex.push_back(i);
            }
            // else {      // TODO: delete this else or comment it after debbuging, is useles in regular work flow
            //     ROS_INFO("There's no missmatched elements");
            // }
        }

        /* Step 2): Obtaining info of the coupled results ==> CASE A, when the vMissmatchedIndex.size() == 0 */
        for (size_t i = 0; i < vCoupledIndex.size(); i++) {
            /* Computing the difference in each axis for each camera and object */
            dCam1XDiff = abs(odZYCam1.vopObsParam[vCoupledIndex[i].first].dFurthestPoint - odZYCam1.vopObsParam[vCoupledIndex[i].first].dClosestPoint);
            dCam1YDiff = abs(odZYCam1.vopObsParam[vCoupledIndex[i].first].dWidthHigher - odZYCam1.vopObsParam[vCoupledIndex[i].first].dWidthLower);
            dCam1ZDiff = abs(odZYCam1.vopObsParam[vCoupledIndex[i].first].dHeightHigher - odZYCam1.vopObsParam[vCoupledIndex[i].first].dHeightLower);;
            dCam2XDiff = abs(odXYCam2.vopObsParam[vCoupledIndex[i].second].dHeightHigher - odXYCam2.vopObsParam[vCoupledIndex[i].second].dHeightLower);
            dCam2YDiff = abs(odXYCam2.vopObsParam[vCoupledIndex[i].second].dWidthHigher - odXYCam2.vopObsParam[vCoupledIndex[i].second].dWidthHigher);
            dCam2ZDiff = abs(odXYCam2.vopObsParam[vCoupledIndex[i].second].dFurthestPoint - odXYCam2.vopObsParam[vCoupledIndex[i].second].dClosestPoint);
            // ROS_INFO("==> Obstacle number: %d", odZYCam1.siObsFound);
            // ROS_INFO("  Camera1: x_diff = %f", dCam1XDiff);
            // ROS_INFO("  Camera2: x_diff = %f", dCam2XDiff);
            // ROS_INFO("  Camera1: y_diff = %f", dCam1YDiff);
            // ROS_INFO("  Camera2: y_diff = %f", dCam2YDiff);
            // ROS_INFO("  Camera1: z_diff = %f", dCam1ZDiff);
            // ROS_INFO("  Camera2: z_diff = %f", dCam2ZDiff);
            /* Picking the optimal solution for the paired obstacles */
            dXdiff = dCam2XDiff;
            if(dCam1YDiff > dCam2YDiff)
                dYdiff = dCam1YDiff;
            else
                dYdiff = dCam2YDiff;
            dZdiff = dCam1ZDiff;
            /* Computing the smallest cube that contain each of the paired obstacles */
            occAux.dXcenter = odZYCam1.vopObsParam[vCoupledIndex[i].first].dClosestPoint + dXdiff / 2;
            occAux.dYcenter = (odZYCam1.vopObsParam[vCoupledIndex[i].first].dWidthCenter + odXYCam2.vopObsParam[vCoupledIndex[i].second].dWidthCenter) / 2;
            occAux.dZcenter = odZYCam1.vopObsParam[vCoupledIndex[i].first].dHeightHigher + dZdiff / 2;
            occAux.dMinCartRadius = sqrt(std::pow(dXdiff/2, 2.0) + std::pow(dYdiff/2, 2.0) + std::pow(dZdiff/2, 2.0));
            occAux.dXdiff = dXdiff;
            occAux.dYdiff = dYdiff;
            occAux.dZdiff = dZdiff;
            ocpVar.occObstaclePoses.push_back(occAux);
            ocpVar.vsiPublishCamera.push_back(1);
            // ROS_INFO("The smallest containing box is at (%f,%f,%f) with a maximum radius of %f", ocpVar.occObstaclePoses[i].dXcenter, ocpVar.occObstaclePoses[i].dYcenter, ocpVar.occObstaclePoses[i].dZcenter, ocpVar.occObstaclePoses[i].dMinCartRadius);
        }

        ROS_INFO("vMissmatchedIndex Value: %d", vMissmatchedIndex.size());
        /* Step 3): Working with the miss-matched results. It means a soft occlusion situation ==> CASE B.1. (double Y side occlusion with non-paired centers) and B.2. (double view occlusion), both are treated the same way */
        if (vMissmatchedIndex.size() != 0){
            std::vector<std::pair <int, int>> vMissCoupledIndex;
            /* Find the possible pairs for the cam1 miss-matched results */
            if (vMissmatchedIndex.size() == 1) {
                // ROS_INFO("The value of vMissmatchedIndex should be one");
                bool bMissMatched;
                /* When there's only one obstacle miss-matched, it must be found the index for cam2 to match this obstacle */
                for (size_t i = 0; i < odXYCam2.siObsFound; i++) {
                    bMissMatched = false;
                    for (size_t j = 0; j < vCoupledIndex.size(); j++) {
                        if (i = vCoupledIndex[j].second)  {
                            bMissMatched = true;
                        }  // It means that that index i is contained in the paired vecto so it has a matched result
                    }
                    if (!bMissMatched) {
                        vMissCoupledIndex.push_back(std::make_pair(vMissmatchedIndex[0], i));
                    }
                }
                // ROS_INFO("Value of vMissCoupledIndex size is: %d", vMissCoupledIndex.size());
            }
            else {
                // ROS_INFO("The value of vMissmatchedIndex should be less than one");
                /* When there's more than only one obstacle miss-matched, it must be found every combination of missmatched obstacles to check which are the closest and more probable */
                std::vector<int> vMissListIndex1 = vMissmatchedIndex, vMissListIndex2;
                bool bMissMatched;
                double dMinDistance, dDistanceComp;
                int iAuxCam1, iAuxCam2;
                /* When there's only one obstacle miss-matched, it must be found the index for cam2 to match this obstacle */
                for (size_t i = 0; i < odXYCam2.siObsFound; i++) {
                    bMissMatched = false;
                    for (size_t j = 0; j < vCoupledIndex.size(); j++) {
                        if (i = vCoupledIndex[j].second)  {
                            bMissMatched = true;
                        }  // It means that that index i is contained in the paired vecto so it has a matched result
                    }
                    if (!bMissMatched) {
                        vMissListIndex2.push_back(i);           // Computing the miss-matched results from camera 2
                    }
                }
                /* Checking every possible combination to pick the closest centers on Y coordinate to be the one matched */
                for (size_t i = 0; i < vMissListIndex1.size(); i++) {
                    dMinDistance = 1000000000.0;
                    iAuxCam1 = 0;
                    iAuxCam2 = 0;
                    for (size_t j = 0; j < vMissListIndex2.size(); j++) {
                        dDistanceComp = abs(odZYCam1.vopObsParam[i].dWidthCenter - odXYCam2.vopObsParam[j].dWidthCenter);
                        if (dMinDistance > dDistanceComp) {
                            dMinDistance = dDistanceComp;
                            iAuxCam1 = vMissListIndex1[i];
                            iAuxCam2 = vMissListIndex2[j];
                        }
                    }
                    vMissCoupledIndex.push_back(std::make_pair(iAuxCam1, iAuxCam2));
                }
            }

            /* After obtainin the couple which presents miss-matching issues, the same algorithm is applied to case B.1. and B.2. */
            for (size_t i = 0; i < vMissCoupledIndex.size(); i++) {
                ROS_INFO("Cycle has entered the vMissCoupledIndex algorithm part");
                /* Computing the difference in each axis for each camera and object */
                dCam1XDiff = abs(odZYCam1.vopObsParam[vMissCoupledIndex[i].first].dFurthestPoint - odZYCam1.vopObsParam[vMissCoupledIndex[i].first].dClosestPoint);
                dCam1YDiff = odZYCam1.vopObsParam[vMissCoupledIndex[i].first].dWidthLower - odZYCam1.vopObsParam[vMissCoupledIndex[i].first].dWidthCenter;
                dCam1ZDiff = abs(odZYCam1.vopObsParam[vMissCoupledIndex[i].first].dHeightHigher - odZYCam1.vopObsParam[vMissCoupledIndex[i].first].dHeightLower);;
                dCam2XDiff = abs(odXYCam2.vopObsParam[vMissCoupledIndex[i].second].dHeightHigher - odXYCam2.vopObsParam[vMissCoupledIndex[i].second].dHeightLower);
                dCam2YDiff = odXYCam2.vopObsParam[vMissCoupledIndex[i].second].dWidthCenter - odXYCam2.vopObsParam[vMissCoupledIndex[i].second].dWidthHigher;
                dCam2ZDiff = abs(odXYCam2.vopObsParam[vMissCoupledIndex[i].second].dFurthestPoint - odXYCam2.vopObsParam[vMissCoupledIndex[i].second].dClosestPoint);
                // ROS_INFO("==> Obstacle number: %d", odZYCam1.siObsFound);
                // ROS_INFO("  Camera1: x_diff = %f", dCam1XDiff);
                // ROS_INFO("  Camera2: x_diff = %f", dCam2XDiff);
                // ROS_INFO("  Camera1: y_diff = %f", dCam1YDiff);
                // ROS_INFO("  Camera2: y_diff = %f", dCam2YDiff);
                // ROS_INFO("  Camera1: z_diff = %f", dCam1ZDiff);
                // ROS_INFO("  Camera2: z_diff = %f", dCam2ZDiff);
                /* Picking the optimal solution for the paired obstacles */
                dXdiff = dCam2XDiff;
                dYdiff = abs(dCam1YDiff + (odZYCam1.vopObsParam[vMissCoupledIndex[i].first].dWidthCenter - odXYCam2.vopObsParam[vMissCoupledIndex[i].second].dWidthCenter) + dCam2YDiff);
                dZdiff = dCam1ZDiff;
                // ROS_INFO("x_diff=%f, y_diff=%f, z_diff=%f", dXdiff, dYdiff, dZdiff);
                /* Computing the smallest cube that contain each of the paired obstacles */
                occAux.dXcenter = odZYCam1.vopObsParam[vMissCoupledIndex[i].first].dClosestPoint + dXdiff / 2;
                occAux.dYcenter = (odZYCam1.vopObsParam[vMissCoupledIndex[i].first].dWidthCenter + odXYCam2.vopObsParam[vMissCoupledIndex[i].second].dWidthCenter) / 2;
                occAux.dZcenter = odZYCam1.vopObsParam[vMissCoupledIndex[i].first].dHeightHigher + dZdiff / 2;
                occAux.dMinCartRadius = sqrt(std::pow(dXdiff/2, 2.0) + std::pow(dYdiff/2, 2.0) + std::pow(dZdiff/2, 2.0));
                occAux.dXdiff = dXdiff;
                occAux.dYdiff = dYdiff;
                occAux.dZdiff = dZdiff;
                ocpVar.occObstaclePoses.push_back(occAux);
                ocpVar.vsiPublishCamera.push_back(1);
                // ROS_INFO("The smallest containing box is at (%f,%f,%f) with a maximum radius of %f", ocpVar.occObstaclePoses[i].dXcenter, ocpVar.occObstaclePoses[i].dYcenter, ocpVar.occObstaclePoses[i].dZcenter, ocpVar.occObstaclePoses[i].dMinCartRadius);
            }
        }

        ocpVar.siNumberObstacles = odZYCam1.siObsFound;
        return ocpVar;
    }

    /* hard_occlusion_xyz_computation() method for cases where the number of obstacles detected by the cam1 and the cam2 differs */
    obs_carthesian_position image_processing::hard_occlusion_xyz_computation() {
        obs_carthesian_position ocpVar;
        obs_cartheseian_components occAux;

        std::vector<std::pair<int, int>> vCoupledIndex;
        std::vector<int> vMissmatchedIndex;

        bool bSameCenterY, bSameLowY, bSameHighY;
        int iMissmatchedResults;
        int iAux = 0;

        double dCam1XDiff = 0.0;
        double dCam2XDiff = 0.0;
        double dCam1YDiff = 0.0;
        double dCam2YDiff = 0.0;
        double dCam1ZDiff = 0.0;
        double dCam2ZDiff = 0.0;
        double dXdiff, dYdiff, dZdiff;

        /* Algorithm for processing obstacles positions when hard occlusion happends */
        if (odZYCam1.siObsFound > odXYCam2.siObsFound) {
            /* Case C.1.: Where there're more obstacles found in cam 1 than in cam2 */
            /* Step 1)  Computing if there are any matched obstacle in both pictures */
            for (size_t i = 0; i < odZYCam1.siObsFound; i++) {
                iMissmatchedResults = 0;
                for (size_t j = 0; j < odXYCam2.siObsFound; j++) {
                    bSameCenterY = false;
                    bSameLowY = false;
                    bSameHighY = false;
                    if (odZYCam1.vopObsParam[i].dWidthCenter >= odXYCam2.vopObsParam[j].dWidthCenter - dDistanceThreshold && odZYCam1.vopObsParam[i].dWidthCenter <= odXYCam2.vopObsParam[j].dWidthCenter + dDistanceThreshold) {
                        bSameCenterY = true;
                    }
                    if (odZYCam1.vopObsParam[i].dWidthLower >= odXYCam2.vopObsParam[j].dWidthLower - dDistanceThreshold && odZYCam1.vopObsParam[i].dWidthLower <= odXYCam2.vopObsParam[j].dWidthLower + dDistanceThreshold) {
                        bSameLowY = true;
                    }
                    if (odZYCam1.vopObsParam[i].dWidthHigher >= odXYCam2.vopObsParam[j].dWidthHigher - dDistanceThreshold && odZYCam1.vopObsParam[i].dWidthHigher <= odXYCam2.vopObsParam[j].dWidthHigher + dDistanceThreshold) {
                        bSameHighY = true;
                    }
                    if (bSameCenterY || bSameLowY || bSameHighY) {
                        // ROS_INFO("    Obstacle matched");
                        vCoupledIndex.push_back(std::make_pair(i, j));
                    }
                    else {
                        // ROS_INFO("    Obstacle miss-matched");
                        iMissmatchedResults++;
                    }
                }
                if (iMissmatchedResults == odXYCam2.siObsFound) {
                    // ROS_INFO("There's a missmatched element");
                    vMissmatchedIndex.push_back(i);
                }
                // else {      // TODO: delete this else or comment it after debbuging, is useles in regular work flow
                //     ROS_INFO("There's no missmatched elements");
                // }
            }    

            /* Step 2) Computation of the box for the missmatched elements */
            /* Working with the paired obstacles for the cam1 */
            for (size_t i = 0; i < vCoupledIndex.size(); i++) {
                /* Computing the difference in each axis for each camera and object */
                dCam1XDiff = abs(odZYCam1.vopObsParam[vCoupledIndex[i].first].dFurthestPoint - odZYCam1.vopObsParam[vCoupledIndex[i].first].dClosestPoint);
                dCam1YDiff = abs(odZYCam1.vopObsParam[vCoupledIndex[i].first].dWidthHigher - odZYCam1.vopObsParam[vCoupledIndex[i].first].dWidthLower);
                dCam1ZDiff = abs(odZYCam1.vopObsParam[vCoupledIndex[i].first].dHeightHigher - odZYCam1.vopObsParam[vCoupledIndex[i].first].dHeightLower);;
                dCam2XDiff = abs(odXYCam2.vopObsParam[vCoupledIndex[i].second].dHeightHigher - odXYCam2.vopObsParam[vCoupledIndex[i].second].dHeightLower);
                dCam2YDiff = abs(odXYCam2.vopObsParam[vCoupledIndex[i].second].dWidthHigher - odXYCam2.vopObsParam[vCoupledIndex[i].second].dWidthHigher);
                dCam2ZDiff = abs(odXYCam2.vopObsParam[vCoupledIndex[i].second].dFurthestPoint - odXYCam2.vopObsParam[vCoupledIndex[i].second].dClosestPoint);
                // ROS_INFO("==> Obstacle number: %d", odZYCam1.siObsFound);
                // ROS_INFO("  Camera1: x_diff = %f", dCam1XDiff);
                // ROS_INFO("  Camera2: x_diff = %f", dCam2XDiff);
                // ROS_INFO("  Camera1: y_diff = %f", dCam1YDiff);
                // ROS_INFO("  Camera2: y_diff = %f", dCam2YDiff);
                // ROS_INFO("  Camera1: z_diff = %f", dCam1ZDiff);
                // ROS_INFO("  Camera2: z_diff = %f", dCam2ZDiff);
                /* Picking the optimal solution for the paired obstacles */
                dXdiff = dCam2XDiff;
                if(dCam1YDiff > dCam2YDiff)                             // This comparisson is for treating the dual-side occlusion or the one-side occlusion on y side.
                    dYdiff = dCam1YDiff;
                else
                    dYdiff = dCam2YDiff;
                dZdiff = dCam1ZDiff;
                /* Computing the smallest cube that contain each of the paired obstacles */
                occAux.dXcenter = odZYCam1.vopObsParam[vCoupledIndex[i].first].dClosestPoint + dXdiff / 2;
                occAux.dYcenter = (odZYCam1.vopObsParam[vCoupledIndex[i].first].dWidthCenter + odXYCam2.vopObsParam[vCoupledIndex[i].second].dWidthCenter) / 2;
                occAux.dZcenter = odZYCam1.vopObsParam[vCoupledIndex[i].first].dHeightHigher + dZdiff / 2;
                occAux.dMinCartRadius = sqrt(std::pow(dXdiff/2, 2.0) + std::pow(dYdiff/2, 2.0) + std::pow(dZdiff/2, 2.0));
                occAux.dXdiff = dXdiff;
                occAux.dYdiff = dYdiff;
                occAux.dZdiff = dZdiff;
                ocpVar.occObstaclePoses.push_back(occAux);
                ocpVar.vsiPublishCamera.push_back(1);
                // ROS_INFO("The smallest containing box is at (%f,%f,%f) with a maximum radius of %f", ocpVar.occObstaclePoses[i].dXcenter, ocpVar.occObstaclePoses[i].dYcenter, ocpVar.occObstaclePoses[i].dZcenter, ocpVar.occObstaclePoses[i].dMinCartRadius);
            }
            if (ocpVar.occObstaclePoses.size() == 0)
                iAux = 0;
            else
                iAux = ocpVar.occObstaclePoses.size() - 1;
            /* Working with the rest and the missmatched results */
            for (size_t i = 0; i < vMissmatchedIndex.size(); i++) {
                /* Computing the difference in each axis for each camera and object */
                dXdiff = abs(odZYCam1.vopObsParam[vMissmatchedIndex[i]].dFurthestPoint - odZYCam1.vopObsParam[vMissmatchedIndex[i]].dClosestPoint);
                dYdiff = abs(odZYCam1.vopObsParam[vMissmatchedIndex[i]].dWidthHigher - odZYCam1.vopObsParam[vMissmatchedIndex[i]].dWidthLower);
                dZdiff = abs(odZYCam1.vopObsParam[vMissmatchedIndex[i]].dHeightHigher - odZYCam1.vopObsParam[vMissmatchedIndex[i]].dHeightLower);
                // ROS_INFO("==> Obstacle number: %d", odZYCam1.siObsFound);
                // ROS_INFO("  Camera1: x_diff = %f", dXdiff);
                // ROS_INFO("  Camera1: y_diff = %f", dYdiff);
                // ROS_INFO("  Camera1: z_diff = %f", dZdiff);
                /* Computing the smallest cube that contain each of the paired obstacles */
                occAux.dXcenter = odZYCam1.vopObsParam[vMissmatchedIndex[i]].dClosestPoint + dXdiff / 2;
                occAux.dYcenter = odZYCam1.vopObsParam[vMissmatchedIndex[i]].dWidthCenter;
                occAux.dZcenter = odZYCam1.vopObsParam[vMissmatchedIndex[i]].dHeightLower + dZdiff / 2;
                occAux.dMinCartRadius = sqrt(std::pow(dXdiff/2, 2.0) + std::pow(dYdiff/2, 2.0) + std::pow(dZdiff/2, 2.0));
                occAux.dXdiff = dXdiff;
                occAux.dYdiff = dYdiff;
                occAux.dZdiff = dZdiff;
                ocpVar.occObstaclePoses.push_back(occAux);
                ocpVar.vsiPublishCamera.push_back(1);
                // ROS_INFO("Print the values of i = %d, iAux = %d", i, iAux);
                // ROS_INFO("The smallest containing box is at (%f,%f,%f) with a maximum radius of %f", ocpVar.occObstaclePoses[i+iAux].dXcenter, ocpVar.occObstaclePoses[i+iAux].dYcenter, ocpVar.occObstaclePoses[i+iAux].dZcenter, ocpVar.occObstaclePoses[i+iAux].dMinCartRadius);
            }
            ocpVar.siNumberObstacles = odZYCam1.siObsFound;
        }
        else {
            /* Case C.2.: Where there're more obstacles found in cam2 than in cam 1 */
            /* Step 1)  Computing if there are any matched obstacle in both pictures */
            for (size_t i = 0; i < odXYCam2.siObsFound; i++) {
                iMissmatchedResults = 0;
                for (size_t j = 0; j < odZYCam1.siObsFound; j++) {
                    bSameCenterY = false;
                    bSameLowY = false;
                    bSameHighY = false;
                    if (odXYCam2.vopObsParam[i].dWidthCenter >= odZYCam1.vopObsParam[j].dWidthCenter - dDistanceThreshold && odXYCam2.vopObsParam[i].dWidthCenter <= odZYCam1.vopObsParam[j].dWidthCenter + dDistanceThreshold) {
                        bSameCenterY = true;
                    }
                    if (odXYCam2.vopObsParam[i].dWidthLower >= odZYCam1.vopObsParam[j].dWidthLower - dDistanceThreshold && odXYCam2.vopObsParam[i].dWidthLower <= odZYCam1.vopObsParam[j].dWidthLower + dDistanceThreshold) {
                        bSameLowY = true;
                    }
                    if (odXYCam2.vopObsParam[i].dWidthHigher >= odZYCam1.vopObsParam[j].dWidthHigher - dDistanceThreshold && odXYCam2.vopObsParam[i].dWidthHigher <= odZYCam1.vopObsParam[j].dWidthHigher + dDistanceThreshold) {
                        bSameHighY = true;
                    }
                    if (bSameCenterY || bSameLowY || bSameHighY) {
                        // ROS_INFO("    Obstacle matched");
                        vCoupledIndex.push_back(std::make_pair(j, i));
                    }
                    else {
                        // ROS_INFO("    Obstacle miss-matched");
                        iMissmatchedResults++;
                        // ROS_INFO("Number of missmatched elements: %d", iMissmatchedResults);
                    }
                }
                if (iMissmatchedResults == odZYCam1.siObsFound) {
                    // ROS_INFO("There's a missmatched element");
                    vMissmatchedIndex.push_back(i);
                }
                // else {      // TODO: delete this else or comment it after debbuging, is useles in regular work flow
                //     ROS_INFO("There's no missmatched elements");
                // }
            }    

            /* Step 2) Computation of the box for the missmatched elements */
            /* Working with the paired obstacles for the cam1 */
            for (size_t i = 0; i < vCoupledIndex.size(); i++) {
                /* Computing the difference in each axis for each camera and object */
                dCam1XDiff = abs(odZYCam1.vopObsParam[vCoupledIndex[i].first].dFurthestPoint - odZYCam1.vopObsParam[vCoupledIndex[i].first].dClosestPoint);
                dCam1YDiff = abs(odZYCam1.vopObsParam[vCoupledIndex[i].first].dWidthHigher - odZYCam1.vopObsParam[vCoupledIndex[i].first].dWidthLower);
                dCam1ZDiff = abs(odZYCam1.vopObsParam[vCoupledIndex[i].first].dHeightHigher - odZYCam1.vopObsParam[vCoupledIndex[i].first].dHeightLower);;
                dCam2XDiff = abs(odXYCam2.vopObsParam[vCoupledIndex[i].second].dHeightHigher - odXYCam2.vopObsParam[vCoupledIndex[i].second].dHeightLower);
                dCam2YDiff = abs(odXYCam2.vopObsParam[vCoupledIndex[i].second].dWidthHigher - odXYCam2.vopObsParam[vCoupledIndex[i].second].dWidthHigher);
                dCam2ZDiff = abs(odXYCam2.vopObsParam[vCoupledIndex[i].second].dFurthestPoint - odXYCam2.vopObsParam[vCoupledIndex[i].second].dClosestPoint);
                // ROS_INFO("==> Obstacle number: %d", odXYCam2.siObsFound);
                // ROS_INFO("  Camera1: x_diff = %f", dCam1XDiff);
                // ROS_INFO("  Camera2: x_diff = %f", dCam2XDiff);
                // ROS_INFO("  Camera1: y_diff = %f", dCam1YDiff);
                // ROS_INFO("  Camera2: y_diff = %f", dCam2YDiff);
                // ROS_INFO("  Camera1: z_diff = %f", dCam1ZDiff);
                // ROS_INFO("  Camera2: z_diff = %f", dCam2ZDiff);
                /* Picking the optimal solution for the paired obstacles */
                dXdiff = dCam2XDiff;
                if(dCam1YDiff > dCam2YDiff)
                    dYdiff = dCam1YDiff;
                else
                    dYdiff = dCam2YDiff;
                dZdiff = dCam1ZDiff;
                /* Computing the smallest cube that contain each of the paired obstacles */
                occAux.dXcenter = odZYCam1.vopObsParam[vCoupledIndex[i].first].dClosestPoint + dXdiff / 2;
                occAux.dYcenter = (odZYCam1.vopObsParam[vCoupledIndex[i].first].dWidthCenter + odXYCam2.vopObsParam[vCoupledIndex[i].second].dWidthCenter) / 2;
                occAux.dZcenter = odZYCam1.vopObsParam[vCoupledIndex[i].first].dHeightHigher + dZdiff / 2;
                occAux.dMinCartRadius = sqrt(std::pow(dXdiff/2, 2.0) + std::pow(dYdiff/2, 2.0) + std::pow(dZdiff/2, 2.0));
                occAux.dXdiff = dXdiff;
                occAux.dYdiff = dYdiff;
                occAux.dZdiff = dZdiff;
                ocpVar.occObstaclePoses.push_back(occAux);
                ocpVar.vsiPublishCamera.push_back(1);
                // ROS_INFO("The smallest containing box is at (%f,%f,%f) with a maximum radius of %f", ocpVar.occObstaclePoses[i].dXcenter, ocpVar.occObstaclePoses[i].dYcenter, ocpVar.occObstaclePoses[i].dZcenter, ocpVar.occObstaclePoses[i].dMinCartRadius);
            }
            if (ocpVar.occObstaclePoses.size() == 0)
                iAux = 0;
            else
                iAux = ocpVar.occObstaclePoses.size() - 1;
            /* Working with the rest and the missmatched results */
            for (size_t i = 0; i < vMissmatchedIndex.size(); i++) {
                /* Computing the difference in each axis for each camera and object */
                dXdiff = abs(odXYCam2.vopObsParam[vMissmatchedIndex[i]].dHeightHigher - odXYCam2.vopObsParam[vMissmatchedIndex[i]].dHeightLower);
                dYdiff = abs(odXYCam2.vopObsParam[vMissmatchedIndex[i]].dWidthHigher - odXYCam2.vopObsParam[vMissmatchedIndex[i]].dWidthHigher);
                dZdiff = abs(odXYCam2.vopObsParam[vMissmatchedIndex[i]].dFurthestPoint - odXYCam2.vopObsParam[vMissmatchedIndex[i]].dClosestPoint);
                // ROS_INFO("==> Obstacle number: %d", odZYCam1.siObsFound);
                // ROS_INFO("  Camera1: x_diff = %f", dXdiff);
                // ROS_INFO("  Camera1: y_diff = %f", dYdiff);
                // ROS_INFO("  Camera1: z_diff = %f", dZdiff);
                /* Computing the smallest cube that contain each of the paired obstacles */
                occAux.dXcenter = odXYCam2.vopObsParam[vMissmatchedIndex[i]].dHeightLower + dXdiff / 2;
                occAux.dYcenter = odXYCam2.vopObsParam[vMissmatchedIndex[i]].dWidthCenter;
                occAux.dZcenter = odXYCam2.vopObsParam[vMissmatchedIndex[i]].dClosestPoint + dZdiff / 2;
                occAux.dMinCartRadius = sqrt(std::pow(dXdiff/2, 2.0) + std::pow(dYdiff/2, 2.0) + std::pow(dZdiff/2, 2.0));
                occAux.dXdiff = dXdiff;
                occAux.dYdiff = dYdiff;
                occAux.dZdiff = dZdiff;
                ocpVar.occObstaclePoses.push_back(occAux);
                ocpVar.vsiPublishCamera.push_back(2);
                // ROS_INFO("Print the values of i = %d, iAux = %d", i, iAux);
                // ROS_INFO("The smallest containing box is at (%f,%f,%f) with a maximum radius of %f", ocpVar.occObstaclePoses[i+iAux].dXcenter, ocpVar.occObstaclePoses[i+iAux].dYcenter, ocpVar.occObstaclePoses[i+iAux].dZcenter, ocpVar.occObstaclePoses[i+iAux].dMinCartRadius);
            }
            ocpVar.siNumberObstacles = odXYCam2.siObsFound;
        }

        return ocpVar;
    }

    /* Obstacle tf2 stamped position publishing method */
    bool image_processing::publish_stamped_obs_poses(obs_carthesian_position ocpVar) {
        static tf2_ros::TransformBroadcaster tbBroadcaster;
        geometry_msgs::TransformStamped tsMessage;
        tf2::Quaternion q;

        try {
            if (ocpVar.siNumberObstacles != 0){
                ros::Time tTime = ros::Time::now();
                for (size_t i = 0; i < ocpVar.siNumberObstacles; i++)
                {
                    tsMessage.header.stamp = tTime;
                    if (ocpVar.vsiPublishCamera[i] == 1) {
                        tsMessage.header.frame_id = "rs_d435_cam_color_optical_frame";
                        tsMessage.transform.translation.x = ocpVar.occObstaclePoses[i].dZcenter;
                        tsMessage.transform.translation.y = ocpVar.occObstaclePoses[i].dYcenter;
                        tsMessage.transform.translation.z = ocpVar.occObstaclePoses[i].dXcenter;
                        q.setRPY(0, 0, 0);
                        tsMessage.transform.rotation.x = q.x();
                        tsMessage.transform.rotation.y = q.y();
                        tsMessage.transform.rotation.z = q.z();
                        tsMessage.transform.rotation.w = q.w();
                    }
                    else {
                        tsMessage.header.frame_id = "rs_d435_cam2_color_optical_frame";
                        tsMessage.transform.translation.x = ocpVar.occObstaclePoses[i].dYcenter;
                        tsMessage.transform.translation.y = ocpVar.occObstaclePoses[i].dXcenter;
                        tsMessage.transform.translation.z = ocpVar.occObstaclePoses[i].dZcenter;
                        q.setRPY(0, 0, 0);
                        tsMessage.transform.rotation.x = q.x();
                        tsMessage.transform.rotation.y = q.y();
                        tsMessage.transform.rotation.z = q.z();
                        tsMessage.transform.rotation.w = q.w();
                    }   
                    tsMessage.child_frame_id = "obstacle_frame_" + std::to_string(i);
                    

                    tbBroadcaster.sendTransform(tsMessage);
                }
            }
        } catch (ros::Exception &re) {
            ROS_ERROR_STREAM("[Exception Catched While Publishing Obs. Pose] - " << re.what());
            return false;
        }
        return true;
    }

    /* Method to publish the point cloud of the obstacles found by each of the cameras */
    bool image_processing::publish_obs_point_cloud() {
        PointCloud::Ptr pcl_msg1 (new PointCloud);
        PointCloud::Ptr pcl_msg2 (new PointCloud);
        sensor_msgs::PointCloud2 pcCloudOut1, pcCloudOut2;
        pcl::PCLPointCloud2::Ptr pcl_aux_msg (new pcl::PCLPointCloud2);

        double x, y, z;

        int counter;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;

        /* Initilization of the headers of both point cloud messages */
        pcl_msg1->header.frame_id = sPubFrame1; 
        pcl_msg2->header.frame_id = sPubFrame2; 

        /* Trying to publish te point cloud obstacles */
        try {
            pcl_msg1->points.clear();
            pcl_msg2->points.clear();
            pcl_msg1->height = 1;
            pcl_msg2->height = 1;
            pcObsPointCloud.data.clear();
            pcCloudOut1.data.clear();
            pcCloudOut2.data.clear();


            if (odZYCam1.siObsFound != 0 || odXYCam2.siObsFound != 0) {
                /* Obtaining the point clouds of the filtered obstacles for each of the cameras */
                counter = 0;
                for (size_t i = 0; i < mDepthDistance1.size().width; i++){
                    for(size_t j=0; j < mDepthDistance1.size().height; j++) {
                        if (mDepthDistance1.at<uint16_t>(cv::Point(i,j)) > iSceneMin && mDepthDistance1.at<uint16_t>(cv::Point(i,j)) < iSceneMax) {
                            z = mDepthDistance1.at<uint16_t>(cv::Point(i,j)) * 0.001;
                            x = (i - dCam1_cx)*z / dCam1_fx;
                            y = (j - dCam1_cy)*z / dCam1_fy;
                            pcl_msg1->points.push_back(pcl::PointXYZ(x,y,z));
                            counter++;
                        }
                    }
                }
                pcl_msg1->width = counter;            
                pcl::toPCLPointCloud2(*pcl_msg1, *pcl_aux_msg);
                pcl_conversions::fromPCL(*pcl_aux_msg, pcCloudOut1);

                counter = 0;
                for (size_t i = 0; i < mDepthDistance2.size().width; i++){
                    for(size_t j=0; j < mDepthDistance2.size().height; j++) {
                        if (mDepthDistance2.at<uint16_t>(cv::Point(i,j)) > iSceneMin && mDepthDistance2.at<uint16_t>(cv::Point(i,j)) < iSceneMax) {
                            z = mDepthDistance2.at<uint16_t>(cv::Point(i,j)) * 0.001;
                            x = (i - dCam1_cx)*z / dCam1_fx;
                            y = (j - dCam1_cy)*z / dCam1_fy;
                            pcl_msg2->points.push_back(pcl::PointXYZ(x,y,z));
                            counter++;
                        }
                    }
                }
                pcl_msg2->width = counter;   
                pcl::toPCLPointCloud2(*pcl_msg2, *pcl_aux_msg);
                pcl_conversions::fromPCL(*pcl_aux_msg, pcCloudOut2); 


                /* Transforming the point cloud of camera 2 to the reference frame of camera 1 */
                try{
                    transformStamped = tfBuffer.lookupTransform("rs_d435_cam_color_optical_frame", "rs_d435_cam2_color_optical_frame", pcCloudOut2.header.stamp, ros::Duration(3.0));
                    tf2::doTransform(pcCloudOut2, pcCloudOut2, transformStamped);
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    return false;
                }

                
                /* Concatenating the point clouds and publishing the result into the desired ROS topic */
                pcl::concatenatePointCloud(pcCloudOut1, pcCloudOut2, pcObsPointCloud);
                pcObsPointCloud.header.frame_id = sPubFrame1;
                pcObsPointCloud.header.stamp = ros::Time::now();
                ROS_DEBUG("Which is of the composed... : %d", pcObsPointCloud.data.size() );
                pcl_conversions::toPCL(pcObsPointCloud, *pcl_aux_msg);
                ROS_INFO("Original number of points: %d", pcl_aux_msg->data.size());
                // pcl_pub.publish(pcObsPointCloud);
            }
            else {
                pcObsPointCloud.header.stamp = ros::Time::now();
                pcObsPointCloud.data.clear();
                pcl_msg1->points.clear();
                pcl_msg1->height = 1;
                pcl_msg1->width = 1;
                pcl_msg1->points.push_back(pcl::PointXYZ(0.0, 0.0, 10.2));
                pcl::toPCLPointCloud2(*pcl_msg1, *pcl_aux_msg);
                // pcl_conversions::fromPCL(*pcl_aux_msg, pcObsPointCloud);
                // pcl_pub.publish(pcObsPointCloud);
            }
            // sensor_msgs::Image imgMessage;
            // pcl::toROSMsg(pcObsPointCloud, imgMessage);
            // pcl_img_pub.publish(imgMessage);
            /* Voxel grid filter */
            // PointCloud::Ptr cloudFiltered;
            // ROS_INFO("It has arrived pre voxel grid filtering");
            pcl::PCLPointCloud2::Ptr cloudFiltered (new pcl::PCLPointCloud2());

            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud(pcl_aux_msg);
            sor.setLeafSize(0.02f, 0.02f, 0.02f);
            sor.filter(*cloudFiltered);
            // ROS_INFO("It has finished voxel grid image filtering");
            pcl_conversions::fromPCL(*cloudFiltered, pcObsPointCloud);
            // ROS_INFO("There's no problem while converting to PCL ROS Message");
            pcl_pub.publish(pcObsPointCloud);
            ROS_INFO("Number of filtered points are: %d", pcObsPointCloud.data.size());
            // ROS_INFO("The voxel grid should be properly published");
            

        } catch (ros::Exception &re) {
            ROS_ERROR_STREAM("[Exception catched shile publishing obstacles point clouds - " << re.what());
            return false;
        }
        return true;
    }

    /***********************************************************
     * ROS CALLBACK MEHTODS OF THE CLASS
     **********************************************************/ 


    /***********************************************************
     * CONSTRUCTOR AND DESTRUCTOR OF THE CLASS
     **********************************************************/ 
    image_processing::image_processing(ros::NodeHandle nh) : icCamGrabber1("1",true),
                                                             icCamGrabber2("2",true) {
        _nh = nh;
    }

    image_processing::~image_processing() {
        if(!cv::getWindowProperty("Camera 2 - RGB Picture",cv::WND_PROP_AUTOSIZE) == -1 || !cv::getWindowProperty("Camera 1 - RGB Picture",cv::WND_PROP_AUTOSIZE) == -1){
            cv::destroyAllWindows();
        }
    }

    /***********************************************************
     *  PUBLIC METHODS OF THE CLASS
     **********************************************************/ 
    /* init() */
    bool image_processing::init() {
        ROS_INFO("Initializing the applications");
        try {
            /* Initialization of the different attributes used by the class. If any cannot be initialized, it will return a false command */
            ciColorInfoPtr1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera1/color/camera_info", _nh);
            ciColorInfo1 = *ciColorInfoPtr1;
            dCam1_fx = ciColorInfo1.K[0];
            dCam1_cx = ciColorInfo1.K[2];
            dCam1_fy = ciColorInfo1.K[4];
            dCam1_cy = ciColorInfo1.K[5];
            ROS_INFO("Storage of the color camera 1 parameters done cx=%f, fx=%f, cy=%f and fy=%f...", dCam1_cx, dCam1_fx, dCam1_cy, dCam1_fy);
            ciColorInfoPtr2 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera2/color/camera_info", _nh);
            ciColorInfo2 = *ciColorInfoPtr2;
            dCam2_fx = ciColorInfo2.K[0];
            dCam2_cx = ciColorInfo2.K[2];
            dCam2_fy = ciColorInfo2.K[4];
            dCam2_cy = ciColorInfo2.K[5];
            ROS_INFO("Storage of the color camera 2 parameters done cx=%f, fx=%f, cy=%f and fy=%f...", dCam2_cx, dCam2_fx, dCam2_cy, dCam2_fy);
            if (icCamGrabber1.isAlignedDepthOn()) {
                // Read the "/camera1/aligned_depth_to_color/camera_info" topic
                ciDepthInfoPtr1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera1/aligned_depth_to_color/camera_info", _nh);
                ciDepthInfo1 = * ciDepthInfoPtr1;
                dCamD1_fx = ciDepthInfo1.K[0];
                dCamD1_cx = ciDepthInfo1.K[2];
                dCamD1_fy = ciDepthInfo1.K[4];
                dCamD1_cy = ciDepthInfo1.K[5];
                ROS_INFO("Storage of the depth camera 1 parameters done cx=%f, fx=%f, cy=%f and fy=%f...", dCamD1_cx, dCamD1_fx, dCamD1_cy, dCamD1_fy);
            }
            else {
                // Read the "/camera1/depth/camera_info" topic
                ciDepthInfoPtr1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera1/depth/camera_info", _nh);
                ciDepthInfo1 = * ciDepthInfoPtr1;
                dCamD1_fx = ciDepthInfo1.K[0];
                dCamD1_cx = ciDepthInfo1.K[2];
                dCamD1_fy = ciDepthInfo1.K[4];
                dCamD1_cy = ciDepthInfo1.K[5];
                ROS_INFO("Storage of the depth camera 1 parameters done cx=%f, fx=%f, cy=%f and fy=%f...", dCamD1_cx, dCamD1_fx, dCamD1_cy, dCamD1_fy);
            }
            if (icCamGrabber2.isAlignedDepthOn()) {
                // Read the "/camera2/aligned_depth_to_color/camera_info" topic
                ciDepthInfoPtr2 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera2/aligned_depth_to_color/camera_info", _nh);
                ciDepthInfo2 = * ciDepthInfoPtr2;
                dCamD2_fx = ciDepthInfo2.K[0];
                dCamD2_cx = ciDepthInfo2.K[2];
                dCamD2_fy = ciDepthInfo2.K[4];
                dCamD2_cy = ciDepthInfo2.K[5];
                ROS_INFO("Storage of the depth camera 1 parameters done cx=%f, fx=%f, cy=%f and fy=%f...", dCamD2_cx, dCamD2_fx, dCamD2_cy, dCamD2_fy);
            }
            else {
                // Read the "/camera2/depth/camera_info" topic
                ciDepthInfoPtr2 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera2/depth/camera_info", _nh);
                ciDepthInfo2 = * ciDepthInfoPtr2;
                dCamD2_fx = ciDepthInfo2.K[0];
                dCamD2_cx = ciDepthInfo2.K[2];
                dCamD2_fy = ciDepthInfo2.K[4];
                dCamD2_cy = ciDepthInfo2.K[5];
                ROS_INFO("Storage of the depth camera 1 parameters done cx=%f, fx=%f, cy=%f and fy=%f...", dCamD2_cx, dCamD2_fx, dCamD2_cy, dCamD2_fy);
            }

            /* TODO: Propper initilization of the processing pictures and auxiliar images */
            
            /* Initiliazation of boolean flags for image processing */
            bColorStored1 = false;
            bColorStored2 = false;
            bDepthStored1 = false;
            bDepthStored2 = false;
            bIsProcessing = false;

            /* Initialization of the cv::Scalar threshold values */
            sLowerThreshold = cv::Scalar(38, 65,65);
            sUpperThreshold = cv::Scalar(87, 255, 255);
            dAreaThreshold = 600;
            dDistanceThreshold = 0.05;
            iKernelSize = 4;

            /* Starting ros_publishing publisher for the Point Cloud publication */
            pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/filtered_aligned_depth_cam1",5);
            pcl_img_pub = _nh.advertise<sensor_msgs::Image>("/camera1/depth_registered_filtered/image_raw",5);
            pcl_info_pub = _nh.advertise<sensor_msgs::CameraInfo>("/camera1/depth_registered_filtered/camera_info",5);
            sPubFrame1 = "rs_d435_cam_color_optical_frame";
            sPubFrame2 = "rs_d435_cam2_color_optical_frame";
            pcObsPointCloud.header.frame_id = sPubFrame1;
            iSceneMin = 20;
            iSceneMax = 4500;
        }
        catch (...) {
            ROS_ERROR("The image processing application cannot be initialized...");
            return false;
        }
        return true;
    }

    /* main run() function */
    void image_processing::run() {
        /* Checking if the application is already running - TODO: check if this step is really neccesary */
        if (bImageProcessingAppRunning) {
            ROS_ERROR("Image distance processing application is already running");
            return;
        }
        bImageProcessingAppRunning = true;

        /* Launching the main application loop */
        try {
            main_loop();
        }
        catch(const std::runtime_error &re) {
            ROS_ERROR_STREAM("Runtime error: " << re.what());
        }
        catch(const std::exception &ex) {
            ROS_ERROR_STREAM("Error occurred: " << ex.what());
        }
        catch(...) {
            ROS_ERROR("Unknown error occurred. Please check connections");
        }

        bImageProcessingAppRunning = false;
    }


    /***********************************************************
     * SETTERS AND GETTERS OF THE CLASS
     **********************************************************/ 
    /* Getters */
    bool image_processing::getbIsProcessing() {
        return bIsProcessing;
    }
    
    void image_processing::showCam1ColorPic() {
        icCamGrabber1.displayImage(1);
    }

    void image_processing::showCam2ColorPic() {
        icCamGrabber2.displayImage(1);
    }

    void image_processing::showCam1DepthPic()  {
        icCamGrabber1.displayImage(4);
    }

    void image_processing::showCam2DepthPic() {
        icCamGrabber1.displayImage(4);
    }

    void image_processing::showCam1ColorProcessedPic() {
        cv::imshow("Camera 1 - RGB Picture", mColorProcessed1);
        // cv::waitKey();
        // cv::destroyAllWindows();
    }

    void image_processing::showCam2ColorProcessedPic() {
        cv::imshow("Camera 2 - RGB Picture", mColorProcessed2);
        // cv::waitKey();
        // cv::destroyAllWindows();
    }

    cv::Scalar image_processing::getsLowerThreshold() {
        return sLowerThreshold;
    }

    cv::Scalar image_processing::getsUpperThreshold() {
        return sUpperThreshold;
    }

    double image_processing::getdAreaThreshold() {
        return dAreaThreshold;
    }

    double image_processing::getdDistanceThreshold() {
        return dDistanceThreshold;
    }

    int image_processing::getErosioniKernelSize() {
        return iKernelSize;
    }

    sensor_msgs::CameraInfo image_processing::getCam1ColorInfo() {
        return ciColorInfo1;
    }

    sensor_msgs::CameraInfo image_processing::getCam2ColorInfo() {
        return ciColorInfo2;
    }

    sensor_msgs::CameraInfo image_processing::getCam1DepthInfo() {
        return ciDepthInfo1;
    }

    sensor_msgs::CameraInfo image_processing::getCam2DepthInfo() {
        return ciDepthInfo2;
    }

    obs_carthesian_position image_processing::getObstaclesXYZPositions() {
        return ocpXYZPositions;
    }

    /* Setters */
    bool image_processing::setsLowerThreshold(cv::Scalar sThreshValue) {
        try {
            sLowerThreshold = sThreshValue;
        } catch (...) {
            return false;
        }
        return true;
    }

    bool image_processing::setsUpperThreshold(cv::Scalar sThreshValue) {
        try {
            sUpperThreshold = sThreshValue;
        } catch (...) {
            return false;
        }
        return true;
    }

    bool image_processing::setdAreaThreshold(double dAreaValue) {
        try {
            dAreaThreshold = dAreaValue;
        } catch (...) {
            return false;
        }
        return true;
    }

    bool image_processing::setdDistanceThreshold(double dDistanceValue) {
        try {
            dDistanceThreshold = dDistanceValue;
        } catch (...) {
            return false;
        }
        return true;
    }

    bool image_processing::setErosioniKernelSize(int iKernelValue) {
        try {
            iKernelSize = iKernelValue;
        } catch (...) {
            return false;
        }
        return true;
    }


    /***********************************************************
     * PROTECTED METHODS OF THE CLASS
     **********************************************************/ 

}