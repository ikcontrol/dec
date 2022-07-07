#include "../../include/vision/image_processing_class.h"
#include <iostream>
#include <chrono>
#include <thread>

namespace pic_handling
{
    /***********************************************************
     * PRIVATE METHODS OF THE CLASS
    **********************************************************/ 
    void image_processing::main_loop() {
        /* Non-cyclic part of the main loop. For example, it can be used for moving the robot to home position (it is not this particular case) */
        double dTime = (double)cv::getTickCount();
        bool First_Image_Prof=false;
        bool First_Image_RGB=false;
        bool First_Prof_No_Robot=false;
        ImageNumber=0;
        
        printf("Begining...\n\n\n");
        printf("*****\n\n\n");
        ros::Duration(4.0).sleep();
      
       
        /* Cyclic part of the main loop */
        while (ros::ok()) 
        {
            if (icCamGrabber1.getbDepthReceived() && icCamGrabber1.getbColorReceived() && !bIsProcessing)
            {
                try
                {
                        listener.waitForTransform("/rs_d435_cam_color_optical_frame", "/tool_center_point", ros::Time(0), ros::Duration(10.0) );
                        listener.lookupTransform("/rs_d435_cam_color_optical_frame", "/base_link",ros::Time(0), base_0);
                        listener.lookupTransform("/rs_d435_cam_color_optical_frame", "/forearm_link",ros::Time(0), forearm);
                        listener.lookupTransform("/rs_d435_cam_color_optical_frame", "/shoulder_link",ros::Time(0), shoulder);
                        listener.lookupTransform("/rs_d435_cam_color_optical_frame", "/wrist_1_link",ros::Time(0), wrist_1);
                        listener.lookupTransform("/rs_d435_cam_color_optical_frame", "/wrist_2_link",ros::Time(0), wrist_2);
                        listener.lookupTransform("/rs_d435_cam_color_optical_frame", "/wrist_3_link",ros::Time(0), wrist_3);
                        listener.lookupTransform("/rs_d435_cam_color_optical_frame", "/tool_center_point",ros::Time(0), tool_center_point);

                }
                catch (tf::TransformException &ex) 
                {
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();
                        
                }
                  
            }
                

            /* First read and store cyclicly the images */
            if(icCamGrabber1.getbColorReceived() && !bIsProcessing)
            {
               
                icCamGrabber1.getmColorImage().copyTo(mColorPic1);
//************************************************************************/ Instantiation to save RGB pic
                using boost::lexical_cast;
                using std::string;
                string finish_nameRGB=".jpg";
                string numstring;
                numstring = lexical_cast<string>(ImageNumber);
                numstring += finish_nameRGB;
                // string nombreArchivo = sHomeDir + "/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
                string nameFileRGB = sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/RGB";
                nameFileRGB += numstring;
                //cv::imwrite(nameFileRGB, mColorPic1);
//***************************************************************************************//////

                mColorPic1.copyTo(ImageRGB2);

// first RGB pic
                if (First_Image_RGB==false)
                {

        // save first pic for reference RGB image
                    mColorPic1.copyTo(ImageRGB);
        // BlackImage to reset processing pics
                    ImageRGB.copyTo(BlackImage);
                    cv::cvtColor(BlackImage, BlackImage, cv::COLOR_BGR2GRAY);
                    // cv::imshow("BlackImage_e", BlackImage );
                    // cv::waitKey(5000);
                    cv::threshold(BlackImage, BlackImage, -1, 255, cv::THRESH_BINARY_INV);
                    // cv::imshow("BlackImage", BlackImage );
                    // cv::waitKey(5000);
                    First_Image_RGB=true;

                } 


                bColorStored1 = true;
                icCamGrabber1.clearbColorReceived();
                
            }

            if(icCamGrabber1.getbDepthReceived() && !bIsProcessing) 
            {
            
                icCamGrabber1.getmDepthImage().copyTo(mDepthPic1);
                // icCamGrabber1.getmDepthImage().copyTo(mDepthDistance1);
                // mDepthDistance1 = mDepthDistance1 * 0.001;

//**************************************************************/ Instantiation to save Depth Pic 
                using boost::lexical_cast;
                using std::string;
                string finish_nameProf=".jpg";
                string numstring;
                numstring = lexical_cast<string>(ImageNumber);
                numstring += finish_nameProf;
                // string nombreArchivo = sHomeDir + "/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
                string nameFileProf = sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/Prof.tiff";
                string nameFileProf_2 = sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/Prof.tiff";
                cv::imwrite(nameFileProf, mDepthPic1);
                nameFileProf_2 += numstring;
//********************************************************************* */  

    // 8 bit Depth pic
                ImagenProf2_U8 = cv::imread(nameFileProf);
    // 16 bit Depth pic
                mDepthPic1.copyTo(ImageProf2);
    // save first pic for reference image
                if (First_Image_Prof==false)
                {
                    mDepthPic1.copyTo(ImageProf);
                    First_Image_Prof=true;
                    ImagenProf_U8 = cv::imread(nameFileProf);

                } 
           
                icCamGrabber1.getmDepthImage().copyTo(mDepthColor1);
                // cv::minMaxLoc(mDepthColor1, &minValue, &maxValue);
    // show Depth pic
                cv::convertScaleAbs(mDepthColor1, mDepthColor1, 0.03);
                cv::applyColorMap(mDepthColor1, mDepthColor1, cv::COLORMAP_JET);
                //cv::imwrite(nameFileProf_2, mDepthColor1);
                bDepthStored1 = true;
                icCamGrabber1.clearbDepthReceived();
        
            }

  
            if(bColorStored1 && bDepthStored1 && !bIsProcessing) 
            {
                bIsProcessing = true;
                bColorStored1 = false;
                bDepthStored1 = false;
                
            }

            /* Processing the stored images for obtaining the detected obstacles positions */
            if(bIsProcessing)
            {
                dTime = (double)cv::getTickCount();
                
                
        

/* Robot filtering segementation algorithm */
    // Calculate the joints position
                Calculate_Tf();
    // obtain forearm - wrist_1 space
                BlackImage.copyTo(forearm_wrist_1);
                link_space( float(x_imageforearm), float(y_imageforearm), float(x_imagewrist_1), float(y_imagewrist_1),forearm_wrist_1);
                // cv::imshow(" forearm_wrist_1", forearm_wrist_1);
                // cv::waitKey(10000);
    // obtain shoulder - forearm (calculated) space
                BlackImage.copyTo(shoulder_forearm_real);
                link_space( float(x_imageshoulder_real), float(y_imageshoulder_real), float(x_imageforearm_real), float(y_imageforearm_real),shoulder_forearm_real);
                // cv::imshow(" shoulder_forearm_real", shoulder_forearm_real);
                // cv::waitKey(10000);

    // obtain wrist_1 - wrist_2 (calculated) space
                BlackImage.copyTo(wrist_1_wrist_2);
                link_space( float(x_imagewrist_1), float(y_imagewrist_1), float(x_imagewrist_2), float(y_imagewrist_2),wrist_1_wrist_2);
                // cv::imshow(" wrist_1_wrist_2", wrist_1_wrist_2);
                // cv::waitKey(10000);

    // obtain wrist_2- wrist_3 space
                BlackImage.copyTo(wrist_2_wrist_3);
                link_space( float(x_imagewrist_2), float(y_imagewrist_2), float(x_imagewrist_3), float(y_imagewrist_3),wrist_2_wrist_3);
                // cv::imshow(" wrist_2_wrist_3", wrist_2_wrist_3);
                // cv::waitKey(10000);

    // obtain wrist_3 - tool_center_point space
                BlackImage.copyTo(wrist_3_tool_center_point);
                link_space( float(x_imagewrist_3), float(y_imagewrist_3), float(x_imagetool_center_point), float(y_imagetool_center_point),wrist_3_tool_center_point);
                // cv::imshow(" wrist_3_tool_center_point", wrist_3_tool_center_point);
                // cv::waitKey(10000);

    // show actual and reference RGB pic
                cv::imshow(" Reference pic", ImageRGB);
                cv::waitKey(1);
                cv::imshow(" Actual pic", ImageRGB2);
                cv::waitKey(1);

    // obtain RGB frame difference  (ResFinSeg)
                segmentation(); 

    // substract reference robot position from ResFinSeg (SegRobot)                                       
                obtain_objects_RGB();

    // get the depth filter pic(SegNoRobot) and obtain the objects (Objects)                                
                RemoveRobot();                                      
                
    // save the first depth filter pic
                if (First_Prof_No_Robot==false)                     
                {
                    Prof_No_Robot.copyTo(Prof_No_Robot_Old);
                    First_Prof_No_Robot=true;
                }
    // substract shadow from objects (Objects)
                shadow(); 

    // obtain frame difference depth filter pic and add to Objects (Objects_scene)                                          
                Prof_segmentation();  
    // identify static and dynamic objets                              
                diff_static_dynamic(ImageNumber); 

    // publish static, dynamic and scene segmentation (actual depth filter pic without dynamic objects)              
                publish_objects(ImageNumber);                       

//*******************/NO NECESSARY /***************************//

                object_contour(ImageNumber);                        //draw the contour of dynamic and static objects in RGB pic

                dTime = ((double)cv::getTickCount() - dTime)/cv::getTickFrequency(); // get and print processing time
                ROS_INFO("Time: %f seconds\n", dTime);
                ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
                //cv::destroyAllWindows();

//************************************************************//
                ImageNumber++;                                      // iteration number
                bIsProcessing=false;                                //  not allow processing
            }
        }

    }



    void image_processing::segmentation()
    {
//***********************************// Instantiation to save ResFinSeg pic
        using boost::lexical_cast;
        using std::string;
        string finish_nameRGB=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameRGB;
        // string nombreArchivo = sHomeDir + "/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
                string nameFileResFinSeg= sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/ResFinSeg";
        nameFileResFinSeg += numstring;
///***************************************////

        cv::Mat ResultDiff,image1grey,ResultDiff_Threshold,kernel,ResultDiff_Mor,image2grey,ResultDiff2,ResultDiff2_Threshold,ResultDiff2_Mor;
        cv::cvtColor(ImageRGB, image1grey, cv::COLOR_BGR2GRAY);
        cv::cvtColor(ImageRGB2, image2grey, cv::COLOR_BGR2GRAY);
        cv::subtract(image1grey, image2grey, ResultDiff);
        // cv::imshow(" ResultDiff ", ResultDiff );
        // cv::waitKey(5000);
        cv::threshold(ResultDiff, ResultDiff_Threshold, 50, 255, cv::THRESH_BINARY);
        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::erode(ResultDiff_Threshold, ResultDiff_Mor, kernel);
        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
        cv::dilate(ResultDiff_Mor, ResultDiff_Mor, kernel);

        cv::subtract(image2grey, image1grey, ResultDiff2);
        cv::threshold(ResultDiff2, ResultDiff2_Threshold, 50, 255, cv::THRESH_BINARY);
        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::erode(ResultDiff2_Threshold, ResultDiff2_Mor, kernel);
        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
        cv::dilate(ResultDiff2_Mor, ResultDiff2_Mor, kernel);

        BlackImage.copyTo(ResFinSeg);

        ResFinSeg=ResultDiff_Mor+ResultDiff2_Mor;
        // cv::namedWindow("Resultado diferencia de color",cv::WINDOW_AUTOSIZE);
        // cv::imshow("Resultado diferencia de color", ResFinSeg );
        // cv::waitKey(5000);
        //cv::imwrite(nameFileResFinSeg, ResFinSeg);
    }



    void image_processing::obtain_objects_RGB()
    {
///********************************************************************/// Instantiation to save SegRobot pic
        using boost::lexical_cast;
        using std::string;
        string finish_nameRGB=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameRGB;
        // string nombreArchivo = sHomeDir + "/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
        

        string nameFileSegRobot= sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/SegRobot";
        nameFileSegRobot+= numstring;
        //cv::imwrite(nameFileSegRobot, SegRobot);
///**************************************************************************///

        cv::Mat kernel,difRobot1,difRobot2,ImageProf_Threshold,ImageProf2_Threshold,ImageProf_Threshold_grey,ImageProf2_Threshold_grey;

 // Reference Depth threshold
        cv::threshold(ImagenProf_U8, ImageProf_Threshold, 2, 255, cv::THRESH_BINARY);
        cv::cvtColor(ImageProf_Threshold, ImageProf_Threshold_grey, cv::COLOR_BGR2GRAY);
        cv::threshold(ImageProf_Threshold_grey, ImageProf_Threshold_grey, 2, 255, cv::THRESH_BINARY);

// actual Depth threshold
        cv::threshold(ImagenProf2_U8, ImageProf2_Threshold, 2, 255, cv::THRESH_BINARY);
        cv::cvtColor(ImageProf2_Threshold, ImageProf2_Threshold_grey, cv::COLOR_BGR2GRAY);
        cv::threshold(ImageProf2_Threshold_grey, ImageProf2_Threshold_grey, 2, 255, cv::THRESH_BINARY);


        // cv::imshow("ImageProf_Threshold_grey", ImageProf_Threshold_grey );
        // cv::waitKey(5000);

// subtract position of the reference robot
        cv::subtract(ResFinSeg, ImageProf_Threshold_grey, difRobot1);
        cv::subtract(ResFinSeg, difRobot1, difRobot1);
        // cv::imshow(" robot posición antigua", difRobot1 );
        // cv::waitKey(5000);

        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
        cv::erode(difRobot1, difRobot1, kernel);
        cv::dilate(difRobot1, difRobot1, kernel);
        cv::dilate(difRobot1, difRobot1, kernel);
        cv::dilate(difRobot1, difRobot1, kernel);
        cv::dilate(difRobot1, difRobot1, kernel);
        cv::dilate(difRobot1, difRobot1, kernel);
        cv::threshold(difRobot1, difRobot1, 5, 255, cv::THRESH_BINARY);
        cv::subtract(ResFinSeg, difRobot1, difRobot1);
        // cv::namedWindow("Quitar robot posición antigua",cv::WINDOW_AUTOSIZE);
        // cv::imshow("Quitar robot posición antigua", difRobot1 );
        // cv::waitKey(5000);

// obtain current position of the robot
        cv::subtract(ResFinSeg, ImageProf2_Threshold_grey, difRobot2);
        cv::subtract(ResFinSeg, difRobot2, difRobot2);
            
        // cv::namedWindow("Diferencia de color de imagen actual",cv::WINDOW_AUTOSIZE);
        // cv::imshow("Diferencia de color de imagen actual", difRobot2 );
        // cv::waitKey(5000);

        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    
        cv::threshold(difRobot2, difRobot2, 5, 255, cv::THRESH_BINARY);

        BlackImage.copyTo(SegRobot);


// add difRobot2 and difRobot1 to get the scene segmentation
        cv::add(difRobot2, difRobot1, SegRobot);
        // cv::namedWindow("Segmentación imagen",cv::WINDOW_AUTOSIZE);
        // cv::imshow("Segmentación imagen", SegRobot);
        // cv::waitKey(1);
        //cv::imwrite(nameFileSegRobot, SegRobot);
    }


        



    void image_processing::link_space(float x1_rect,float y1_rect,float x2_rect,float y2_rect,cv::Mat ImagenSolucion )
    {
        float rampa,x_rect,y_rect,x_rect_desp,y_rect_desp;
        bool direccion=true, direccionX=false,direccionY=false;

        rampa=((y2_rect-y1_rect)/(x2_rect-x1_rect));
        
// Calculate distance between two point            

        x_rect_desp= float(x2_rect)-float(x1_rect);
        y_rect_desp= float(y2_rect)-float(y1_rect);

// calculate where is the X2 point respect X1
        if(x_rect_desp>10)
        {
            
            x_rect=x_rect_desp;
            direccion=true;
        }
        else if(x_rect_desp<-10)
        {
            
            x_rect=x_rect_desp;  
            direccion=false;
        }
        else
        {
           
            x_rect=0; 
            direccionX=true;
        }

// calculate where is the Y2 point respect Y1
        if(y_rect_desp<-10)
        {
           
            y_rect=y_rect_desp;
        }
        else if(y_rect_desp>10)
        {
            
            y_rect=y_rect_desp;
        }
        else
        {
            
            y_rect=0;
            direccionY=true;
        }


        if (y2_rect<=0)
        {
            y2_rect=0;
        }
        if (y1_rect<=0)
        {
            y1_rect=0;
        }

        if (x1_rect>640)
        {
            x1_rect=640-1;
        }
        if (x2_rect>640)
        {
            x2_rect=640-1;
        }

// draws possible area of th link
        if (direccion==true and direccionY==false and direccionX==false)
        {
            

            if(x1_rect+35>640)
            {
                x1_rect=x1_rect+(35-x1_rect);
            }
            if(y1_rect-30<0)
            {
                y1_rect=y1_rect+(30-y1_rect);
            }


            for (size_t i = int(x1_rect); i <= int(x1_rect+x_rect) ; i++)
            { 
                
                y_rect=rampa*(float(i)-x1_rect)+y1_rect;
                for (size_t k=i-35; k<i+35;k++)
                {
                    
                    for (size_t n = int(y_rect)-30; n < int(y_rect+30) ; n++)
                    {
                       ImagenSolucion.at<uint8_t>(cv::Point(k,n))=255;
                    }
                }
            }
        }
        else if (direccion==false and direccionY==false and direccionX==false)
        {
            
            if(x1_rect-35<0)
            {
                x1_rect=x1_rect+(35-x1_rect);
            }
            if(y1_rect-20<0)
            {
                y1_rect=y1_rect+(20-y1_rect);
            }

            for (size_t i = int(x1_rect); i >= int(x1_rect+x_rect) ; i=i-1)
            { 
                
                y_rect=rampa*(float(i)-x1_rect)+y1_rect;
                
                
                for (size_t k=i-35; k<i+35;k++)
                {
                    
                    for (size_t n = int(y_rect)-35; n < int(y_rect+35) ; n++)
                    {
                       

                        ImagenSolucion.at<uint8_t>(cv::Point(k,n))=255;
                    }
                }
            }
        }
        else if ( direccionX==true and direccionY==false)
        {
           
            if(y1_rect-y2_rect>0)
            {
                
                if
                (y2_rect-35<0)
                {
                    y2_rect=y2_rect+(35-y2_rect);
                }

                if(x1_rect-30<0)
                {
                    x1_rect=x1_rect+(30-x1_rect);
                }

                for (size_t k=x1_rect-30; k<=x1_rect+x_rect+30;k++)
                {
                    for (size_t n = int(y2_rect)-35; n < int(y1_rect+35) ; n++)
                    {
                        

                        ImagenSolucion.at<uint8_t>(cv::Point(k,n))=255;
                    }
                }
            }
            else
            {
                
                if(y1_rect-35<0)
                {
                    y1_rect=y1_rect+(35-y2_rect);
                } 

                for (size_t k=x1_rect-30; k<=x1_rect+x_rect+30;k++)
                {
                    for (size_t n = int(y1_rect)-35; n < int(y2_rect+35) ; n++)
                    {
                        
                        ImagenSolucion.at<uint8_t>(cv::Point(k,n))=255;
                    }
                }
            }
        }
        else if ( direccionY==true and direccionX==false)
        {
            
            if(x1_rect-x2_rect>0)
            {
            
                
                if (x2_rect-20<0)
                {
                    x2_rect=x2_rect+(20-x2_rect);
                } 

                for (size_t k=x2_rect-30; k<=x1_rect+30;k++)
                {
                    for (size_t n = int(y2_rect)-20; n < int(y1_rect+20) ; n++)
                    {
                        ImagenSolucion.at<uint8_t>(cv::Point(k,n))=255;
                    }
                }
            }
            else
            {
                if (x1_rect-20<0)
                {
                    x1_rect=x1_rect+(20-x1_rect);
                } 

        
                for (size_t k=x1_rect-30; k<x2_rect+30;k++)
                {
                    for (size_t n = int(y1_rect)-20; n <= int(y2_rect+20) ; n++)
                    {
                        ImagenSolucion.at<uint8_t>(cv::Point(k,n))=255;
                    }
                }
            }
        }
    }



    void image_processing:: Calculate_Tf()
    {
    /* New values */
        Eigen::Matrix3f R2,R3,R4;
        Eigen:: Matrix<float, 3, 1> Pc2,Pc3,Pc4,d2,d3,d4,shoulder_Vect,forearm_Vect,wrist_1_Vect;

        double ValueX3_real; 
        double ValueY3_real;
        double ValueZ3_real;

        double ValueX2_real;
        double ValueY2_real;
        double ValueZ2_real;

        double ValueX4_real; 
        double ValueY4_real;
        double ValueZ4_real;

// calculate TFs joints position in the image
        z_imageBase=base_0.getOrigin().z()/0.001;
        x_imageBase=((base_0.getOrigin().x()*dCam1_fx)/base_0.getOrigin().z())+ dCam1_cx;
        y_imageBase=((base_0.getOrigin().y()*dCam1_fy)/base_0.getOrigin().z())+dCam1_cy; 

        z_imageforearm=forearm.getOrigin().z()/0.001;
        x_imageforearm=((forearm.getOrigin().x()*dCam1_fx)/forearm.getOrigin().z())+ dCam1_cx;
        y_imageforearm=((forearm.getOrigin().y()*dCam1_fy)/forearm.getOrigin().z())+dCam1_cy;

        z_imageshoulder=shoulder.getOrigin().z()/0.001;
        x_imageshoulder=((shoulder.getOrigin().x()*dCam1_fx)/shoulder.getOrigin().z())+ dCam1_cx;
        y_imageshoulder=((shoulder.getOrigin().y()*dCam1_fy)/shoulder.getOrigin().z())+dCam1_cy; 

        z_imagewrist_1=wrist_1.getOrigin().z()/0.001;
        x_imagewrist_1=((wrist_1.getOrigin().x()*dCam1_fx)/wrist_1.getOrigin().z())+ dCam1_cx;
        y_imagewrist_1=((wrist_1.getOrigin().y()*dCam1_fy)/wrist_1.getOrigin().z())+dCam1_cy; 

        z_imagewrist_2=wrist_2.getOrigin().z()/0.001;
        x_imagewrist_2=((wrist_2.getOrigin().x()*dCam1_fx)/wrist_2.getOrigin().z())+ dCam1_cx;
        y_imagewrist_2=((wrist_2.getOrigin().y()*dCam1_fy)/wrist_2.getOrigin().z())+dCam1_cy;

        z_imagewrist_3=wrist_3.getOrigin().z()/0.001;
        x_imagewrist_3=((wrist_3.getOrigin().x()*dCam1_fx)/wrist_3.getOrigin().z())+ dCam1_cx;
        y_imagewrist_3=((wrist_3.getOrigin().y()*dCam1_fy)/wrist_3.getOrigin().z())+dCam1_cy;

        z_imagetool_center_point=tool_center_point.getOrigin().z()/0.001;
        x_imagetool_center_point=((tool_center_point.getOrigin().x()*dCam1_fx)/tool_center_point.getOrigin().z())+ dCam1_cx;
        y_imagetool_center_point=((tool_center_point.getOrigin().y()*dCam1_fy)/tool_center_point.getOrigin().z())+dCam1_cy;
        
//get the parameters to calculate the computed values
        R2 = Eigen::Quaternionf(shoulder.getRotation().w(), shoulder.getRotation().x(), shoulder.getRotation().y(), shoulder.getRotation().z()).toRotationMatrix();
        Pc2 <<  shoulder.getOrigin().x(),
                shoulder.getOrigin().y(),
                shoulder.getOrigin().z();

        R3 = Eigen::Quaternionf(forearm.getRotation().w(), forearm.getRotation().x(), forearm.getRotation().y(), forearm.getRotation().z()).toRotationMatrix();
       

        Pc3 <<  forearm.getOrigin().x(),
                forearm.getOrigin().y(),
                forearm.getOrigin().z();

        

        R4 = Eigen::Quaternionf(wrist_1.getRotation().w(), wrist_1.getRotation().x(), wrist_1.getRotation().y(), wrist_1.getRotation().z()).toRotationMatrix();
        

        Pc4 <<  wrist_1.getOrigin().x(),
                wrist_1.getOrigin().y(),
                wrist_1.getOrigin().z();


        d2 <<   0,
                -0.176,
                0;

        d3 <<   0,
                0,
                0.137;
        
        d4 <<   0,
                0,
                -0.135;

        shoulder_Vect=Pc2+(R2*d2);
        forearm_Vect=Pc3+(R3*d3);
        wrist_1_Vect=Pc4+(R4*d4);
       
        ValueX3_real = double(shoulder_Vect.coeff(0, 0));
        ValueY3_real= double(shoulder_Vect.coeff(1, 0));
        ValueZ3_real= double(shoulder_Vect.coeff(2, 0));

      
        ValueX2_real = double(forearm_Vect.coeff(0, 0));
        ValueY2_real= double(forearm_Vect.coeff(1, 0));
        ValueZ2_real= double(forearm_Vect.coeff(2, 0));

        ValueX4_real = double(wrist_1_Vect.coeff(0, 0));
        ValueY4_real= double(wrist_1_Vect.coeff(1, 0));
        ValueZ4_real= double(wrist_1_Vect.coeff(2, 0));


        /* Calculation of the computed values */
        z_imageforearm_real=ValueZ2_real/0.001;
        x_imageforearm_real=((ValueX2_real*dCam1_fx)/ValueZ2_real)+ dCam1_cx;
        y_imageforearm_real=((ValueY2_real*dCam1_fy)/ValueZ2_real)+dCam1_cy;

        z_imageshoulder_real=ValueZ3_real/0.001;
        x_imageshoulder_real=((ValueX3_real*dCam1_fx)/ValueZ3_real)+ dCam1_cx;
        y_imageshoulder_real=((ValueY3_real*dCam1_fy)/ValueZ3_real)+dCam1_cy; 

        z_imagewrist_1_real=ValueZ4_real/0.001;
        x_imagewrist_1_real=((ValueX4_real*dCam1_fx)/ValueZ4_real)+ dCam1_cx;
        y_imagewrist_1_real=((ValueY4_real*dCam1_fy)/ValueZ4_real)+dCam1_cy; 
    }



    void image_processing::RemoveRobot()
    {
        cv::Mat mDepthDistance = cv::Mat::zeros(cv::Size(ImageRGB2.rows, ImageRGB2.cols), CV_16UC1);
        ImageProf2.copyTo(mDepthDistance);

// remove the pics
        BlackImage.copyTo(Prof_No_Robot);
        BlackImage.copyTo(SegNoRobot);
        BlackImage.copyTo(Objects);
        // cv::imshow("Prof_No_Robot", Prof_No_Robot);
        // cv::waitKey(10000);


        for (size_t i = 0; i < mDepthDistance.size().width; i++)
        {
            for(size_t j=0; j < mDepthDistance.size().height; j++) 
            {
                
        // forearm
                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageforearm-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageforearm+70) 
                {
                    if((i>x_imageforearm and i<x_imageforearm+70 and y_imageforearm+30>j and y_imageforearm-30<j) or (i<x_imageforearm and i>x_imageforearm-70 and  y_imageforearm+30>j and y_imageforearm-30<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }
                    
        // shoulder
                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageshoulder-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageshoulder+70) 
                {
                    if((i>x_imageshoulder and i<x_imageshoulder+70 and y_imageshoulder+50>j and y_imageshoulder-50<j) or (i<x_imageshoulder+70 and i>x_imageshoulder-70 and  y_imageshoulder+50>j and y_imageshoulder-50<j))
                    {
                       Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }


        // calculate shoulder
                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageshoulder_real-150 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageshoulder_real+150) 
                {
                    if((i>x_imageshoulder_real and i<x_imageshoulder_real+70 and y_imageshoulder_real+50>j and y_imageshoulder_real-50<j) or (i<x_imageshoulder_real+70 and i>x_imageshoulder_real-70 and  y_imageshoulder_real+50>j and y_imageshoulder_real-50<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }

        // calculate forearm
                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageforearm_real-140 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageforearm_real+140) 
                {
                    if((i>x_imageforearm_real and i<x_imageforearm_real+70 and y_imageforearm_real+50>j and y_imageforearm_real-50<j) or (i<x_imageforearm_real+70 and i>x_imageforearm_real-70 and  y_imageforearm_real+50>j and y_imageforearm_real-50<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }

        // calculate wrist_1
                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_1_real-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_1_real+120) 
                {
                    if((i>x_imagewrist_1_real and i<x_imagewrist_1_real+70 and y_imagewrist_1_real+50>j and y_imagewrist_1_real-50<j) or (i<x_imagewrist_1_real+70 and i>x_imagewrist_1_real-70 and  y_imagewrist_1_real+50>j and y_imagewrist_1_real-50<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }


        // wrist_1
                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_1-100 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_1+100) 
                {
                    if((i>x_imagewrist_1 and i<x_imagewrist_1+70 and y_imagewrist_1+30> j and y_imagewrist_1-30<j) or (i<x_imagewrist_1 and i>x_imagewrist_1-70 and  y_imagewrist_1+30>j and y_imagewrist_1-30<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;

                    }
                }

        // wrist_2
                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_2-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_2+120) 
                {
                    if((i>x_imagewrist_2 and i<x_imagewrist_2+70 and y_imagewrist_2+50>j and y_imagewrist_2-40<j) or (i<x_imagewrist_2 and i>x_imagewrist_2-70 and  y_imagewrist_2+50>j and y_imagewrist_2-40<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }
        // wrist_3
                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_3-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_3+120) 
                {
                    if((i>x_imagewrist_3 and i<x_imagewrist_3+70 and y_imagewrist_3+30>j and y_imagewrist_3-30<j) or (i<x_imagewrist_3 and i>x_imagewrist_3-70 and  y_imagewrist_3+30>j and y_imagewrist_3-30<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }

        //tool_center_point
                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagetool_center_point-150 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagetool_center_point+150) 
                {
                    if((i>x_imagetool_center_point and i<x_imagetool_center_point+70 and y_imagetool_center_point+50>j and y_imagetool_center_point-30<j) or (i<x_imagetool_center_point and i>x_imagetool_center_point-70 and  y_imagetool_center_point+50>j and y_imagetool_center_point-30<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }

    
    /* Meassures between the joints */
        // shoulder -forearm
                if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageforearm_real-130 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageshoulder_real+120) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageforearm_real+130 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageshoulder_real-120))                                                  
                {
                    if (shoulder_forearm_real.at<uint8_t>(cv::Point(i,j))==255)
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }


        // forearm-wrist_1
                if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageforearm-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_1+100) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageforearm+120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_1-100))                                                   
                {
                    if (forearm_wrist_1.at<uint8_t>(cv::Point(i,j))==255)
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }    
                    

        // wrist_1-wrist_2
                if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_2-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_1+120) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_2+120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_1-120))                                                   
                {
                    if (wrist_1_wrist_2.at<uint8_t>(cv::Point(i,j))==255)
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }


            // wrist_2-wrist_3
                if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_2-140 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_3+140) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_2+140 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_3-140))                                                   
                {
                    if (wrist_2_wrist_3.at<uint8_t>(cv::Point(i,j))==255)
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }


        // wrist_3-tool_center_point
                if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagetool_center_point-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_3+130) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagetool_center_point+120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_3-130))                                                   
                {
                    if (wrist_3_tool_center_point.at<uint8_t>(cv::Point(i,j))==255)
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }


                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))<=300)
                {
                    Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                }
            }
        }

//************************************************************// Instantiation to save Prof_No_Robot and Prof_No_Robotmor pics
        using boost::lexical_cast;
        using std::string;
        string finish_nameRGB=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameRGB;
        // string nombreArchivo = sHomeDir + "/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
        
        string nameFileProf_No_Robot= sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/Prof_No_Robot";
        string nameFileProf_No_Robot_mor= sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/Prof_No_Robotmor";
        nameFileProf_No_Robot+= numstring;
        nameFileProf_No_Robot_mor+= numstring;
//****************************************************************/

        cv::Mat kernel;
        // cv::imshow("Quitar Robot de la imagen de profundidad", Prof_No_Robot); /// antes ImagenNegro11
        // cv::waitKey(1);

// remove robot from the SegRobot pic and save in SegNoRobot
        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
        cv::dilate(Prof_No_Robot, Prof_No_Robot, kernel);
        cv::erode(Prof_No_Robot, Prof_No_Robot, kernel);

        // cv::imshow("Quitar Robot de la imagen de profundidad erode-dilate", Prof_No_Robot); /// antes ImagenNegro11
        // cv::waitKey(1);
        //cv::imwrite(nameFileProf_No_Robot, Prof_No_Robot);



        cv::subtract(SegRobot,Prof_No_Robot,SegNoRobot);// imagennegro2
        // cv::imshow("Quitar robot de la imagen segmentada ", SegNoRobot);
        // cv::waitKey(10000);
        //cv::imwrite(nameFileProf_No_Robot_mor, Prof_No_Robot);


        ImageProf2.copyTo(mDepthDistance);
        std:: vector<double> v = { z_imageBase, z_imageforearm, z_imageshoulder, z_imagewrist_1, z_imagewrist_2, z_imagewrist_3, z_imagetool_center_point};  
        auto result = minmax_element(v.begin(), v.end()); 
        bool RobotFlag=false;

// check that in the image SegNoRobot there are no parts of the robot
        for (size_t i = 0; i < mDepthDistance.size().width; i++)
        {
            for(size_t j=0; j < mDepthDistance.size().height; j++) 
            {

    //pixels detected as object in SegNoRobot
                if (SegNoRobot.at<uint8_t>(cv::Point(i,j))==255)
                {
            // pixels between the nearest joint and the camera detect as object
                    if (mDepthDistance.at<uint16_t>(cv::Point(i,j))<*result.first-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>20 )
                    {
                        Objects.at<uint8_t>(cv::Point(i,j))=255;
                       
                    }
                    else
                    {
            // Base            

                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageBase and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageBase) 
                        {
                            
                        }
            //forearm
                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageforearm-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageforearm+120) 
                        {
                            if((i>x_imageforearm and i<x_imageforearm+50 and y_imageforearm+30>j and y_imageforearm-30<j) or (i<x_imageforearm and i>x_imageforearm-50 and  y_imageforearm+30>j and y_imageforearm-30<j))
                            {
                                RobotFlag=true;
                                
                            }
                        }


            //  calculate shoulder    

                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageshoulder_real-125 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageshoulder_real+125) 
                        {
                            if((i>x_imageshoulder_real and i<x_imageshoulder_real+70 and y_imageshoulder_real+50>j and y_imageshoulder_real-50<j) or (i<x_imageshoulder_real+70 and i>x_imageshoulder_real-70 and  y_imageshoulder_real+50>j and y_imageshoulder_real-50<j))
                            {
                                RobotFlag=true;
                            }
                        }

            //shoulder                
                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageshoulder-100 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageshoulder+100) 
                        {
                            if((i>x_imageshoulder and i<x_imageshoulder+60 and y_imageshoulder+30>j and y_imageshoulder-30<j) or (i<x_imageshoulder and i>x_imageshoulder-60 and  y_imageshoulder+30>j and y_imageshoulder-30<j))
                            {
                                
                                RobotFlag=true;
                               
                            }
                        }
            // wrist_1
                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_1-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_1+70) 
                        {
                            if((i>x_imagewrist_1 and i<x_imagewrist_1+60 and y_imagewrist_1+30> j and y_imagewrist_1-30<j) or (i<x_imagewrist_1 and i>x_imagewrist_1-60 and  y_imagewrist_1+30>j and y_imagewrist_1-30<j))
                            {
                                
                                RobotFlag=true;
                                
                            }
                        }
            // wrist_2
                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_2-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_2+70) 
                        {
                            if((i>x_imagewrist_2 and i<x_imagewrist_2+40 and y_imagewrist_2+30>j and y_imagewrist_2-30<j) or (i<x_imagewrist_2 and i>x_imagewrist_2-40 and  y_imagewrist_2+30>j and y_imagewrist_2-30<j))
                            {
                                RobotFlag=true;
                                
                            }
                        }
            // wrist_3
                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_3-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_3+70) 
                        {
                            if((i>x_imagewrist_3 and i<x_imagewrist_3+50 and y_imagewrist_3+30>j and y_imagewrist_3-30<j) or (i<x_imagewrist_3 and i>x_imagewrist_3-50 and  y_imagewrist_3+30>j and y_imagewrist_3-30<j))
                            {
                                RobotFlag=true;
                                
                            }
                        }
            //tool_center_point
                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagetool_center_point-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagetool_center_point+120) 
                        {                   
                            if((i>x_imagetool_center_point and i<x_imagetool_center_point+50 and y_imagetool_center_point+50>j and y_imagetool_center_point-30<j) or (i<x_imagetool_center_point and i>x_imagetool_center_point-50 and  y_imagetool_center_point+30>j and y_imagetool_center_point-30<j))
                            {
                                RobotFlag=true;
                                
                            }
                        }

    
     /* Meassures between singularities */

            // shoulder -forearm
                        if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageforearm_real-170 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageshoulder_real+170) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageforearm_real+170 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageshoulder_real-170))                                                  
                        {
                            if (shoulder_forearm_real.at<uint8_t>(cv::Point(i,j))==255)
                            {
                                RobotFlag=true;
                                
                            }
                        }


            // forearm-wrist_1
                        if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageforearm-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_1+70) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageforearm+70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_1-70))                                                   
                        {
                            if (forearm_wrist_1.at<uint8_t>(cv::Point(i,j))==255)
                            {
                                RobotFlag=true;
                            }
                        }
        

            // wrist_1-wrist_2
                        if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_2-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_1+70) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_2+70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_1-70))                                                   
                        {
                            if (wrist_1_wrist_2.at<uint8_t>(cv::Point(i,j))==255)
                            {
                                RobotFlag=true;
                            }
                        }


            // wrist_2-wrist_3
                        if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_2-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_3+70) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_2+70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_3-70))                                                   
                        {
                            if (wrist_2_wrist_3.at<uint8_t>(cv::Point(i,j))==255)
                            {
                                RobotFlag=true;
                            }
                        }


            // wrist_3-tool_center_point
                        if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagetool_center_point-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_3+70) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagetool_center_point+120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_3-70))                                                   
                        {
                            if (wrist_3_tool_center_point.at<uint8_t>(cv::Point(i,j))==255)
                            {
                               RobotFlag=true;
                            }
                        }

                                    
                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))<=300)
                        {
                            RobotFlag=true;
                            
                        }
                    
                        if (RobotFlag==false)
                        {
                            Objects.at<uint8_t>(cv::Point(i,j))=255;
                          
                        }
                    
                        RobotFlag=false;
                    }
                }
                    
            }
        }

// Morfology operation to the objects 
        // cv::imshow("Quitando 2 vez robot objetos", Objects);
        // cv::waitKey(10000);

        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
        cv::erode(Objects, Objects, kernel);

        // cv::imshow("Objetos erosión ", Objects);
        // cv::waitKey(10000);
        cv::dilate(Objects, Objects, kernel);

        // cv::imshow("Objects 1 ", Objects);
        // cv::waitKey(1);
    }


    void image_processing::shadow()
    {
// remove the Shadow_Image
        BlackImage.copyTo(Shadow_Image);
// check if there are shadows (comparing pixels in real and reference image depth ) 
        for (size_t i = 0; i < Objects.size().width; i++)
        {
            for(size_t j=0; j < Objects.size().height; j++) 
            {
                if (Objects.at<uint8_t>(cv::Point(i,j))==255)
                {
                    if ((ImageProf.at<uint16_t>(cv::Point(i,j))>=((ImageProf2.at<uint16_t>(cv::Point(i,j)))-100)) and (ImageProf.at<uint16_t>(cv::Point(i,j))<=((ImageProf2.at<uint16_t>(cv::Point(i,j)))+100)))
                    {
                        Shadow_Image.at<uint8_t>(cv::Point(i,j))=255;
                        
                        
                    }
                }       
            }
        }
        // cv::imshow("Shadow_Image", Shadow_Image);
        // cv::waitKey(10000);

// remove shadow  
        cv:: subtract(Objects,Shadow_Image,Objects);
        // cv::imshow("Objects", Objects);
        // cv::waitKey(10000);
    }

   
    void image_processing:: Prof_segmentation()
    {
        cv::Mat kernel;
        BlackImage.copyTo(Objects_scene);
        // cv::imshow("Prof_No_Robot_Old", Prof_No_Robot_Old);
        // cv::waitKey(10000);
        // cv::imshow("Prof_No_Robot", Prof_No_Robot);
        // cv::waitKey(10000);
       
// get the difference between filtered robot images (current and reference)
        cv::subtract(Prof_No_Robot_Old,Prof_No_Robot,Prof_seg_Image);
        // cv::imshow("Prof_segmentation", Prof_seg_Image);
        // cv::waitKey(10000);


        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(20, 20));
        cv::erode(Prof_seg_Image, Prof_seg_Image, kernel);
        cv::dilate(Prof_seg_Image, Prof_seg_Image, kernel);
        // cv::imshow("Prof_seg_Image", Prof_seg_Image);
        // cv::waitKey(10000);

// add the object depth segmenation and RGB object segmentation
        cv:: add(Prof_seg_Image,Objects,Objects_scene);
        // cv::imshow("Objects_scene", Objects_scene);
        // cv::waitKey(10000);

///*********************************************************************/// Instantiation to save Objects_scene pic
        using boost::lexical_cast;
        using std::string;
        string finish_nameRGB=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameRGB;
                // string nombreArchivo = sHomeDir + "/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
        string nameFileObjects_scene= sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/Objects_scene";
    
        nameFileObjects_scene += numstring;
        //cv::imwrite(nameFileObjects_scene, Objects_scene);
////*******************************************************************************///
    }




    void image_processing:: object_contour(int cont)
    {
        cv::Mat canny_Objects_scene,canny_static_object,canny_dynamic_object ;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat Objects_scene_RGB = ImageRGB2.clone();
        cv::Mat static_object_RGB = ImageRGB2.clone();
        cv::Mat dynamic_object_RGB = ImageRGB2.clone();

////*******************************************************************////// Instantiation to save static and dynamic contour pics
        using boost::lexical_cast;
        using std::string;
        string finish_nameRGB=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameRGB;
        // string nombreArchivo = sHomeDir + "/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
        string nameFilestatic_object_RGB= sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/static_object_RGB";
        string nameFiledynamic_object_RGB= sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/dynamic_object_RGB";
        nameFilestatic_object_RGB += numstring;
        nameFiledynamic_object_RGB+= numstring;
////************************************************************************///

// scene contour
        cv::Canny( Objects_scene, canny_Objects_scene, 100, 200 );
        cv::findContours( canny_Objects_scene, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
        cv::drawContours(Objects_scene_RGB, contours, -1, cv::Scalar(0, 255, 0), 2);
        // cv::imshow("Objects_scene_RGB", Objects_scene_RGB);
        // cv::waitKey(1);

// if static and dynamic identification is done
        if (cont>=BufferSize)
        {
    // static contour

            cv::Canny( static_object, canny_static_object, 100, 200 );
            cv::findContours( canny_static_object, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
            cv::drawContours(static_object_RGB, contours, -1, cv::Scalar(255, 0, 0), 2);
            cv::imshow("static_object_RGB", static_object_RGB);
            cv::waitKey(1);
            //cv::imwrite(nameFilestatic_object_RGB, static_object_RGB);

         
    // dynamic contour

            cv::Canny( dynamic_object, canny_dynamic_object, 100, 200 );
            cv::findContours( canny_dynamic_object, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
            cv::drawContours(dynamic_object_RGB, contours, -1, cv::Scalar(0, 0, 255), 2);
            cv::imshow("dynamic_object_RGB", dynamic_object_RGB);
            cv::waitKey(1);
            //cv::imwrite(nameFiledynamic_object_RGB, dynamic_object_RGB);
        }
    }

    
    void image_processing:: diff_static_dynamic(int cont)
    {
        cv::Mat image_Buffer, image_buffer_res,imageMoment,kernel;
        bool First_Flag=false;
        int Buf_num=0;

///**********************************************************/// To save static object contour
        using boost::lexical_cast;
        using std::string;
        string finish_nameProf=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameProf;
        string nameFileProf = sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/static";
        nameFileProf += numstring;
////****************************************************************////

        Objects_scene.copyTo(imageMoment);
        
// if static and dynamic identification is done
        if(cont>=BufferSize)
        {
            Buf_num=0;
            for (const cv::Mat& i :ObjectsSceneBuffer) 
            {
            // first comparation

                        if (First_Flag==false)
                        {
                            cv::bitwise_and(imageMoment,i,image_buffer_res,imageMoment);
                            // cv::imshow("image_buffer_res 1", image_buffer_res );
                            // cv::waitKey(5000);
                            First_Flag=true;
                        }
            // others comparation

                        else
                        {
                            // cv::imshow("imagen i", i);
                            // cv::waitKey(10000);

                            // cv::imshow("imagen imp", image_buffer_res);
                            // cv::waitKey(10000);

                            cv::bitwise_and(i,image_buffer_res,image_Buffer,i);
                            // cv::imshow("imp 3", image_Buffer );
                            // cv::waitKey(5000);
                            image_Buffer.copyTo(image_buffer_res);
                            BlackImage.copyTo(image_Buffer);


                        }
                                    
                        Buf_num++;
            }

            // cv::imshow(" image_buffer_res", image_buffer_res);
            // cv::waitKey(10000);
    // save all the pics of the buffer except the oldest one
            ObjectsSceneBuffer.assign (ObjectsSceneBuffer.begin()+1,ObjectsSceneBuffer.end());
    // save the actual pic on the buffer
            ObjectsSceneBuffer.push_back(imageMoment); 
            //cv::imwrite(nameFileProf, image_buffer_res); 


            Buf_num=0;
            
    // copy the static obeject obtained
            image_buffer_res.copyTo(static_object);

            kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::erode(static_object, static_object, kernel);
            cv::dilate(static_object, static_object, kernel);
    // obtain dynamics objects
            cv::subtract(Objects_scene,static_object,dynamic_object);
    
            kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
            cv::erode(dynamic_object, dynamic_object, kernel);
            cv::dilate(dynamic_object, dynamic_object, kernel);
            // cv::imshow("dynamic_object", dynamic_object);
            // cv::waitKey(1);
            // cv::imshow("static_object", static_object);
            // cv::waitKey(1);
        }
// if the static and dynamic identification is not done 
        if(cont<BufferSize)
        {
                //printf("guarda imagen menor de buff\n");
    // save the pic in the buffer
                ObjectsSceneBuffer.push_back(imageMoment); 
        }
    }


   
    void image_processing::publish_objects(int cont)
    {
        cv::Mat prof_image,scene_image,static_prof,dynamic_prof,scene_prof,image_static_prof,image_dynamic_prof,image_scene_prof,scene_static,scene_static_prof,copyDynamic,kernel;
        sensor_msgs::Image img_msg_scene,img_msg_static,img_msg_dynamic;
        std_msgs::Header Header; // empty header
        sensor_msgs::CameraInfo all_camera_info;
// copy aligned camera info
        all_camera_info=ciDepthInfo1;
//////*****************************************************************///  Instantiation to save pics
        using boost::lexical_cast;
        using std::string;
        string finish_nameProf=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameProf;
        
        string nameFileStatic = sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/static_env";
        nameFileStatic += numstring;
        string nameFileDynamic = sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/dynamic_env";
        nameFileDynamic += numstring;
        string nameFileScene = sHomeDir + "/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/scene_env";
        nameFileScene += numstring;
////*******************************************************************//////

        // header.seq=counter;
// get pic info
        Header.stamp=ros::Time::now();
        Header.frame_id = "rs_d435_cam_color_optical_frame";
// copy actual depth and filter pic    
        ImageProf2.copyTo(prof_image);
        Prof_No_Robot.copyTo(scene_image);

// obtain scene segmentation(dynamic and static) 
        cv::threshold(scene_image, scene_image, 40, 255, cv::THRESH_BINARY_INV);
 // obtain depth scene segmentation(dynamic and static)
        cv::bitwise_and(prof_image,prof_image,scene_prof,scene_image);
        cv_bridge::CvImage img_bridge;
// if dynamic and static identification is not done
        if (cont<BufferSize)
        {
        // convert depth scene segmentation pic to sensor_msgs type
            img_bridge = cv_bridge::CvImage(Header, sensor_msgs::image_encodings::TYPE_16UC1, scene_prof);
            img_bridge.toImageMsg(img_msg_scene);
        // publish depth scene segmentation pic
            scene_pub.publish(img_msg_scene);
        // publish depth scene segmentation camera info
            scene_pub_info.publish(all_camera_info);
        //show depth scene segmentation pic
            cv::convertScaleAbs(scene_prof, image_scene_prof, 0.03);
            cv::applyColorMap(image_scene_prof, image_scene_prof, cv::COLORMAP_JET);
            cv::namedWindow("scene_prof",cv::WINDOW_AUTOSIZE);
            cv::imshow("scene_prof", image_scene_prof );
            cv::waitKey(1);
        }


// if dynamic and static identification is  done
        if(cont>BufferSize)
        {
    // static_object
        // obtain depth static_object 
                cv::bitwise_and(prof_image,prof_image,static_prof,static_object);
        // convert depth static_object pic to sensor_msgs type
                img_bridge = cv_bridge::CvImage(Header, sensor_msgs::image_encodings::TYPE_16UC1, static_prof);
                img_bridge.toImageMsg(img_msg_static);
        // publish depth static_object pic
                static_pub.publish(img_msg_static);
        // publish depth static_object camera info
                static_pub_info.publish(all_camera_info);
        // show depth static_object
                cv::convertScaleAbs(static_prof, image_static_prof, 0.03);
                cv::applyColorMap(image_static_prof, image_static_prof, cv::COLORMAP_JET);
                cv::namedWindow("static_prof",cv::WINDOW_AUTOSIZE);
                cv::imshow("static_prof", image_static_prof );
                cv::waitKey(1);
                //cv::imwrite(nameFileStatic, image_static_prof); 

    // dynamic_object
        // obtain depth dynamic_object 
                cv::Mat dynamic;
                dynamic_object.copyTo(dynamic);
                cv::bitwise_and(prof_image,prof_image,dynamic_prof,dynamic);
        // convert depth dynamic_object pic to sensor_msgs type
                img_bridge = cv_bridge::CvImage(Header, sensor_msgs::image_encodings::TYPE_16UC1, dynamic_prof);
                img_bridge.toImageMsg(img_msg_dynamic);
        // publish depth dynamic_object pic
                dynamic_pub.publish(img_msg_dynamic);
        // publish depth dynamic_object camera info
                dynamic_pub_info.publish(all_camera_info);
        // show depth dynamic_object
                cv::convertScaleAbs(dynamic_prof, image_dynamic_prof, 0.03);
                cv::applyColorMap(image_dynamic_prof, image_dynamic_prof, cv::COLORMAP_JET);
                cv::namedWindow("dynamic_object",cv::WINDOW_AUTOSIZE);
                cv::imshow("dynamic_object", image_dynamic_prof );
                cv::waitKey(1);
                //cv::imwrite(nameFileDynamic, image_dynamic_prof); 


    //scene segmentation
        //get dynamic and scene pics
                Prof_No_Robot.copyTo(scene_image);
                dynamic_object.copyTo(copyDynamic);
        // substract dynamic objets from scene segmentation
                kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
                cv::dilate(copyDynamic, copyDynamic, kernel);
                cv::threshold(scene_image, scene_image, 40, 255, cv::THRESH_BINARY_INV);
                cv::subtract(scene_image,copyDynamic,scene_static);
                // cv::imshow("scene_static", scene_static );
                // cv::waitKey(1);
        // convert depth scene segmentation pic to sensor_msgs type
                cv::bitwise_and(prof_image,prof_image,scene_static_prof,scene_static);
                img_bridge = cv_bridge::CvImage(Header, sensor_msgs::image_encodings::TYPE_16UC1, scene_static_prof);
                img_bridge.toImageMsg(img_msg_scene);
         // publish depth scene segmentation pic
                scene_pub.publish(img_msg_scene);
        // publish depth scene segmentation camera info
                scene_pub_info.publish(all_camera_info);
        //show depth scene segmentation pic
                cv::convertScaleAbs(scene_static_prof, image_scene_prof, 0.03);
                cv::applyColorMap(image_scene_prof, image_scene_prof, cv::COLORMAP_JET);
                cv::namedWindow("scene_prof",cv::WINDOW_AUTOSIZE);
                cv::imshow("scene_prof", image_scene_prof );
                cv::waitKey(1);
                //cv::imwrite(nameFileScene, image_scene_prof); 
        }
        
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
    

    /* main run() function */
    void image_processing::run() 
    {
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
                // Read the "/camera1/aligned_depth_to_color/camera_info" topic
                ciDepthInfoPtr2 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera2/aligned_depth_to_color/camera_info", _nh);
                ciDepthInfo2 = * ciDepthInfoPtr2;
                dCamD2_fx = ciDepthInfo2.K[0];
                dCamD2_cx = ciDepthInfo2.K[2];
                dCamD2_fy = ciDepthInfo2.K[4];
                dCamD2_cy = ciDepthInfo2.K[5];
                ROS_INFO("Storage of the depth camera 1 parameters done cx=%f, fx=%f, cy=%f and fy=%f...", dCamD2_cx, dCamD2_fx, dCamD2_cy, dCamD2_fy);
            }
            else {
                // Read the "/camera1/depth/camera_info" topic
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
            bDepthStored1 = false;
            bIsProcessing = false;

            // Buffer for static objects
            BufferSize=20;

            /* Starting ros_publishing publisher for the Point Cloud publication */
           static_pub = _nh.advertise<sensor_msgs::Image>("/camera1/depth_workpieces/image",5);
           dynamic_pub = _nh.advertise<sensor_msgs::Image>("/camera1/depth_obstacles/image",5);
           scene_pub = _nh.advertise<sensor_msgs::Image>("/camera1/depth_scene/image",5);
           static_pub_info=_nh.advertise<sensor_msgs::CameraInfo>("/camera1/depth_workpieces/camera_info",5);
           dynamic_pub_info = _nh.advertise<sensor_msgs::CameraInfo>("/camera1/depth_obstacles/camera_info",5);
           scene_pub_info = _nh.advertise<sensor_msgs::CameraInfo>("/camera1/depth_scene/camera_info",5);


           /* Reading environment variables to be used for automatic pathing */
           sHomeDir.clear();
           sHomeDir = std::getenv("HOME");
           ROS_ERROR("Printing read $HOME directory: %s", sHomeDir.c_str());
        }
        catch (...) {
            ROS_ERROR("The image processing application cannot be initialized...");
            return false;
        }
        return true;
    }


}
