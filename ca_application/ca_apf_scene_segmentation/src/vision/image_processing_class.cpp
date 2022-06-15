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
                // using boost::lexical_cast;
                // using std::string;
                // string finish_name=".txt";
                // string numstring;
                // numstring = lexical_cast<string>(ImageNumber);
                // numstring += finish_name;


                // // string nombreArchivo = "/home/ikerlan/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
                // string nameFile = "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/Valores";

                // nameFile += numstring;

                // std::ofstream archivo;  
                // archivo.open(nameFile.c_str(), std::fstream::out);


                // archivo << "//Valores Base\n";
                // archivo << "ValorX1=";

                // archivo << lexical_cast<string>(base_0.getOrigin().x());
                // archivo << ";";		
                // archivo << std::endl;
                // archivo << "ValorY1=";
                // archivo << lexical_cast<string>(base_0.getOrigin().y());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "ValorZ1=";
                // archivo << lexical_cast<string>(base_0.getOrigin().z());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << std::endl;


                // archivo << "//Valores forearm\n";
                // archivo << "ValorX2=";
                // archivo << lexical_cast<string>(forearm.getOrigin().x());
                // archivo << ";";		
                // archivo << std::endl;
                // archivo << "ValorY2=";
                // archivo << lexical_cast<string>(forearm.getOrigin().y());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "ValorZ2=";
                // archivo << lexical_cast<string>(forearm.getOrigin().z());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "Valor2rotX=";
                // archivo << lexical_cast<string>(forearm.getRotation().x());	
                // archivo << ";";	
                // archivo << std::endl;
                // archivo << "Valor2rotY=";
                // archivo << lexical_cast<string>(forearm.getRotation().y());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "Valor2rotZ=";
                // archivo << lexical_cast<string>(forearm.getRotation().z());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "Valor2rotW=";
                // archivo << lexical_cast<string>(forearm.getRotation().w());
                // archivo << ";";
                // archivo << std::endl;




                // archivo << std::endl;



                // archivo << "//Valores shoulder\n";
                // archivo << "ValorX3=";

                // archivo << lexical_cast<string>(shoulder.getOrigin().x());	
                // archivo << ";";	
                // archivo << std::endl;
                // archivo << "ValorY3=";
                // archivo << lexical_cast<string>(shoulder.getOrigin().y());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "ValorZ3=";
                // archivo << lexical_cast<string>(shoulder.getOrigin().z());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "Valor3rotX=";
                // archivo << lexical_cast<string>(shoulder.getRotation().x());
                // archivo << ";";		
                // archivo << std::endl;
                // archivo << "Valor3rotY=";
                // archivo << lexical_cast<string>(shoulder.getRotation().y());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "Valor3rotZ=";
                // archivo << lexical_cast<string>(shoulder.getRotation().z());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "Valor3rotW=";
                // archivo << lexical_cast<string>(shoulder.getRotation().w());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << std::endl;


                // archivo << "//Valores wrist_1\n";
                // archivo << "ValorX4=";

                // archivo << lexical_cast<string>(wrist_1.getOrigin().x());
                // archivo << ";";		
                // archivo << std::endl;
                // archivo << "ValorY4=";
                // archivo << lexical_cast<string>(wrist_1.getOrigin().y());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "ValorZ4=";
                // archivo << lexical_cast<string>(wrist_1.getOrigin().z());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "Valor4rotX=";
                // archivo << lexical_cast<string>(wrist_1.getRotation().x());
                // archivo << ";";		
                // archivo << std::endl;
                // archivo << "Valor4rotY=";
                // archivo << lexical_cast<string>(wrist_1.getRotation().y());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "Valor4rotZ=";
                // archivo << lexical_cast<string>(wrist_1.getRotation().z());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "Valor4rotW=";
                // archivo << lexical_cast<string>(wrist_1.getRotation().w());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << std::endl;


                // archivo << "//Valores wrist_2\n";
                // archivo << "ValorX5=";

                // archivo << lexical_cast<string>(wrist_2.getOrigin().x());
                // archivo << ";";		
                // archivo << std::endl;
                // archivo << "ValorY5=";
                // archivo << lexical_cast<string>(wrist_2.getOrigin().y());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "ValorZ5=";
                // archivo << lexical_cast<string>(wrist_2.getOrigin().z());
                // archivo << ";";	
                // archivo << std::endl;
                // archivo << std::endl;


                // archivo << "//Valores wrist_3\n";
                // archivo << "ValorX6=";

                // archivo << lexical_cast<string>(wrist_3.getOrigin().x());
                // archivo << ";";		
                // archivo << std::endl;
                // archivo << "ValorY6=";
                // archivo << lexical_cast<string>(wrist_3.getOrigin().y());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "ValorZ6=";
                // archivo << lexical_cast<string>(wrist_3.getOrigin().z());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << std::endl;

                // archivo << "//Valores tool_center_point\n";
                // archivo << "ValorX7=";

                // archivo << lexical_cast<string>(tool_center_point.getOrigin().x());
                // archivo << ";";		
                // archivo << std::endl;
                // archivo << "ValorY7=";
                // archivo << lexical_cast<string>(tool_center_point.getOrigin().y());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << "ValorZ7=";
                // archivo << lexical_cast<string>(tool_center_point.getOrigin().z());
                // archivo << ";";
                // archivo << std::endl;
                // archivo << std::endl;


                // archivo << "Valores camara\n";
                // archivo << "dCam1_fx=";
                // archivo << lexical_cast<string>(dCam1_fx);
                // archivo << ";";		
                // archivo << std::endl;
                // archivo << "dCam1_cx=";
                // archivo << lexical_cast<string>(dCam1_cx);
                // archivo << ";";		
                // archivo << std::endl;
                // archivo << "dCam1_fy=";
                // archivo << lexical_cast<string>(dCam1_fy);
                // archivo << ";";		
                // archivo << std::endl;
                // archivo << "dCam1_cy=";
                // archivo << lexical_cast<string>(dCam1_cy);
                // archivo << ";";		
                // archivo << std::endl;



                //     // Finalmente lo cerramos
                // archivo.close();
                // std::cout << "Escrito correctamente";   
            }
                

            /* First read and store cyclicly the images */
            if(icCamGrabber1.getbColorReceived() && !bIsProcessing)
            {
               
                icCamGrabber1.getmColorImage().copyTo(mColorPic1);
                using boost::lexical_cast;
                using std::string;
                string finish_nameRGB=".jpg";
                string numstring;
                numstring = lexical_cast<string>(ImageNumber);
                numstring += finish_nameRGB;
                // string nombreArchivo = "/home/ikerlan/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
                string nameFileRGB = "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_application/ca_apf_scene_segmentation/scripts/folder_save/RGB";
                nameFileRGB += numstring;
                //cv::imwrite(nameFileRGB, mColorPic1);
    
                mColorPic1.copyTo(ImageRGB2);

                if (First_Image_RGB==false)
                {
                    mColorPic1.copyTo(ImageRGB);
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


                using boost::lexical_cast;
                using std::string;
                string finish_nameProf=".jpg";
                string numstring;
                numstring = lexical_cast<string>(ImageNumber);
                numstring += finish_nameProf;
                // string nombreArchivo = "/home/ikerlan/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
                string nameFileProf = "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_application/ca_apf_scene_segmentation/scripts/Prof.tiff";
                string nameFileProf_2 = "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_application/ca_apf_scene_segmentation/scripts/folder_save/Prof";
                cv::imwrite(nameFileProf, mDepthPic1);
                nameFileProf_2 += numstring;
                
                ImagenProf2_U8 = cv::imread(nameFileProf);

                mDepthPic1.copyTo(ImageProf2);

                if (First_Image_Prof==false)
                {
                    mDepthPic1.copyTo(ImageProf);
                    First_Image_Prof=true;
                    ImagenProf_U8 = cv::imread(nameFileProf);

                } 
           

           
                
                icCamGrabber1.getmDepthImage().copyTo(mDepthColor1);
                // cv::minMaxLoc(mDepthColor1, &minValue, &maxValue);
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
                
                
                /* Depth Data of each Reference Frame in the picture */
                /* base_0 */
                // profundidaImagenBase=base_0.getOrigin().z()/0.001;
                // x_imagenBase=((base_0.getOrigin().x()*dCam1_fx)/base_0.getOrigin().z())+ dCam1_cx  ;
                // y_imagenBase=((base_0.getOrigin().y()*dCam1_fy)/base_0.getOrigin().z())+dCam1_cy ;
                                            
                /* forearm */
                // profundidaImagenforearm=forearm.getOrigin().z()/0.001;
                // x_imagenforearm= ((forearm.getOrigin().x()*dCam1_fx)/forearm.getOrigin().z())+ dCam1_cx  ;
                // y_imagenforearm=((forearm.getOrigin().y()*dCam1_fy)/forearm.getOrigin().z())+dCam1_cy ;

                
                /* shoulder */
                // profundidaImagenshoulder=shoulder.getOrigin().z()/0.001;
                // x_imagenshoulder= ((shoulder.getOrigin().x()*dCam1_fx)/shoulder.getOrigin().z())+ dCam1_cx  ;
                // y_imagenshoulder=((shoulder.getOrigin().y()*dCam1_fy)/shoulder.getOrigin().z())+dCam1_cy ;

                
                /* wrist_1 */
                // profundidaImagenwrist_1=wrist_1.getOrigin().z()/0.001;
                // x_imagenwrist_1= ((wrist_1.getOrigin().x()*dCam1_fx)/wrist_1.getOrigin().z())+ dCam1_cx  ;
                // y_imagenwrist_1=((wrist_1.getOrigin().y()*dCam1_fy)/wrist_1.getOrigin().z())+dCam1_cy ;

                
                /* wrist_2 */
                // profundidaImagenwrist_2=wrist_2.getOrigin().z()/0.001;
                // x_imagenwrist_2= ((wrist_2.getOrigin().x()*dCam1_fx)/wrist_2.getOrigin().z())+ dCam1_cx  ;
                // y_imagenwrist_2=((wrist_2.getOrigin().y()*dCam1_fy)/wrist_2.getOrigin().z())+dCam1_cy ;

                /* wrist_3 */
                // profundidaImagenwrist_3=wrist_3.getOrigin().z()/0.001;
                // x_imagenwrist_3= (( wrist_3.getOrigin().x()*dCam1_fx)/wrist_3.getOrigin().z())+ dCam1_cx  ;
                // y_imagenwrist_3=((wrist_3.getOrigin().y()*dCam1_fy)/wrist_3.getOrigin().z())+dCam1_cy ;
                
                /* tool_center_point */
                // profundidaImagentool_center_point=tool_center_point.getOrigin().z()/0.001;
                // x_imagentool_center_point= ((tool_center_point.getOrigin().x()*dCam1_fx)/tool_center_point.getOrigin().z())+ dCam1_cx  ;
                // y_imagentool_center_point=((tool_center_point.getOrigin().y()*dCam1_fy)/tool_center_point.getOrigin().z())+dCam1_cy ;


                /* Show The Reference Frame computed position in the console */
                // std::cout.precision(std::numeric_limits<double>::max_digits10 - 1);
                // printf("\nValores Tf en camara Base\n");
                // printf("Valor X:");
                // std::cout << std::scientific <<  x_imagenBase << '\n';
                // printf("Valor Y:");
                // std::cout << std::scientific <<  y_imagenBase << '\n';
                // printf("Valor Z:");
                // std::cout << std::scientific <<  profundidaImagenBase << '\n';

                // std::cout.precision(std::numeric_limits<double>::max_digits10 - 1);
                // printf("\nValores Tf en camara forearm\n");
                // printf("Valor X:");
                // std::cout << std::scientific << x_imagenforearm << '\n';
                // printf("Valor Y:");
                // std::cout << std::scientific <<  y_imagenforearm << '\n';
                // printf("Valor Z:");
                // std::cout << std::scientific <<  profundidaImagenforearm<< '\n';

            
                // std::cout.precision(std::numeric_limits<double>::max_digits10 - 1);
                // printf("\nValores Tf en camara shoulder\n");
                // printf("Valor X:");
                // std::cout << std::scientific <<  x_imagenshoulder << '\n';
                // printf("Valor Y:");
                // std::cout << std::scientific <<  y_imagenshoulder << '\n';
                // printf("Valor Z:");
                // std::cout << std::scientific <<  profundidaImagenshoulder << '\n';


                // std::cout.precision(std::numeric_limits<double>::max_digits10 - 1);
                // printf("\nValores Tf en camara wrist_1\n");
                // printf("Valor X:");
                // std::cout << std::scientific << x_imagenwrist_1 << '\n';
                // printf("Valor Y:");
                // std::cout << std::scientific <<  y_imagenwrist_1<< '\n';
                // printf("Valor Z:");
                // std::cout << std::scientific <<  profundidaImagenwrist_1 << '\n';

                
                // std::cout.precision(std::numeric_limits<double>::max_digits10 - 1);
                // printf("\nValores Tf en camara wrist_2\n");
                // printf("Valor X:");
                // std::cout << std::scientific << x_imagenwrist_2 << '\n';
                // printf("Valor Y:");
                // std::cout << std::scientific <<  y_imagenwrist_2 << '\n';
                // printf("Valor Z:");
                // std::cout << std::scientific <<  profundidaImagenwrist_2 << '\n';


                // std::cout.precision(std::numeric_limits<double>::max_digits10 - 1);
                // printf("\nValores Tf en camara wrist_3\n");
                // printf("Valor X:");
                // std::cout << std::scientific <<  x_imagenwrist_3<< '\n';
                // printf("Valor Y:");
                // std::cout << std::scientific <<  y_imagenwrist_3 << '\n';
                // printf("Valor Z:");
                // std::cout << std::scientific <<  profundidaImagenwrist_3 << '\n';

                // std::cout.precision(std::numeric_limits<float>::max_digits10 - 5);
                // printf("\nValores Tf en camara tool_center_point\n");
                // printf("Valor X:");
                // std::cout << std::scientific <<  x_imagentool_center_point << '\n';
                // printf("Valor Y:");
                // std::cout << std::scientific <<  y_imagentool_center_point << '\n';
                // printf("Valor Z:");
                // std::cout << std::scientific <<  profundidaImagentool_center_point<< '\n';


                // double z = mDepthPic1.at<uint16_t>(cv::Point(int(x_imagenBase),int(y_imagenBase)));
                // printf("Valor de pixel:");
                // std::cout << z << '\n';


                /* Copy pictures to display the reference frames without damaging the original picture */
                // cv::Mat cop1,cop2,cop3,cop4,cop5,cop6,cop7;
                // mColorPic1.copyTo(cop1);
                // mColorPic1.copyTo(cop2);
                // mColorPic1.copyTo(cop3);
                // mColorPic1.copyTo(cop4);
                // mColorPic1.copyTo(cop5);
                // mColorPic1.copyTo(cop6);
                // mColorPic1.copyTo(cop7);


                /* Displaying the reference frames in the picture */
                // cv::circle(cop7,cv::Point(int(x_imagenBase),int(y_imagenBase)),50,(0,0,255));
                // cv::namedWindow("Base",cv::WINDOW_AUTOSIZE);
                // cv::imshow("Base", cop7);
                // cv::waitKey(5000);

                // cv::circle(cop1,cv::Point(int(x_imagenforearm),int(y_imagenforearm)),50,(0,0,255));
                // cv::namedWindow("forearm",cv::WINDOW_AUTOSIZE);
                // cv::imshow("forearm", cop1);
                // cv::waitKey(5000);

                // cv::circle(cop2,cv::Point(int(x_imagenshoulder),int(y_imagenshoulder)),50,(0,0,255));
                // cv::namedWindow("shoulder",cv::WINDOW_AUTOSIZE);
                // cv::imshow("shoulder", cop2);
                // cv::waitKey(5000);

                // cv::circle(cop3,cv::Point(int(x_imagenwrist_1),int(y_imagenwrist_1)),50,(0,0,255));
                // cv::namedWindow("wrist_1",cv::WINDOW_AUTOSIZE);
                // cv::imshow("wrist_1", cop3);
                // cv::waitKey(5000);

                // cv::circle(cop4,cv::Point(int(x_imagenwrist_2),int(y_imagenwrist_2)),50,(0,0,255));
                // cv::namedWindow("wrist_2",cv::WINDOW_AUTOSIZE);
                // cv::imshow("wrist_2", cop4);
                // cv::waitKey(5000);

                // cv::circle(cop5,cv::Point(int(x_imagenwrist_3),int(y_imagenwrist_3)),50,(0,0,255));
                // cv::namedWindow("wrist_3",cv::WINDOW_AUTOSIZE);
                // cv::imshow("wrist_3", cop5);
                // cv::waitKey(5000);

                // cv::circle(cop6,cv::Point(int(x_imagentool_center_point),int(y_imagentool_center_point)),50,(0,0,255));
                // cv::namedWindow("tool_center_point",cv::WINDOW_AUTOSIZE);
                // cv::imshow("tool_center_point", cop6);
                // cv::waitKey(5000);


                /* Robot filtering segementation algorithm */
                Calculate_Tf();

                BlackImage.copyTo(forearm_wrist_1);
                entre_articulaciones( float(x_imageforearm), float(y_imageforearm), float(x_imagewrist_1), float(y_imagewrist_1),forearm_wrist_1);
                // cv::imshow(" forearm_wrist_1", forearm_wrist_1);
                // cv::waitKey(10000);

                BlackImage.copyTo(shoulder_forearm_real);
                entre_articulaciones( float(x_imageshoulder_real), float(y_imageshoulder_real), float(x_imageforearm_real), float(y_imageforearm_real),shoulder_forearm_real);
                // cv::imshow(" shoulder_forearm_real", shoulder_forearm_real);
                // cv::waitKey(10000);

                BlackImage.copyTo(wrist_1_wrist_2);
                entre_articulaciones( float(x_imagewrist_1), float(y_imagewrist_1), float(x_imagewrist_2), float(y_imagewrist_2),wrist_1_wrist_2);
                // cv::imshow(" wrist_1_wrist_2", wrist_1_wrist_2);
                // cv::waitKey(10000);

                BlackImage.copyTo(wrist_2_wrist_3);
                entre_articulaciones( float(x_imagewrist_2), float(y_imagewrist_2), float(x_imagewrist_3), float(y_imagewrist_3),wrist_2_wrist_3);
                // cv::imshow(" wrist_2_wrist_3", wrist_2_wrist_3);
                // cv::waitKey(10000);

                BlackImage.copyTo(wrist_3_tool_center_point);
                entre_articulaciones( float(x_imagewrist_3), float(y_imagewrist_3), float(x_imagetool_center_point), float(y_imagetool_center_point),wrist_3_tool_center_point);
                // cv::imshow(" wrist_3_tool_center_point", wrist_3_tool_center_point);
                // cv::waitKey(10000);

                cv::imshow(" imagen antigua", ImageRGB);
                cv::waitKey(1);

                cv::imshow(" Imagen actual", ImageRGB2);
                cv::waitKey(1);


                segmentar();
                ObtenerObjetosRGB();
                RemoveRobot();
                if (First_Prof_No_Robot==false)
                {
                    Prof_No_Robot.copyTo(Prof_No_Robot_Old);
                    //Prof_background();
                    First_Prof_No_Robot=true;
                }
                shadow();
                Prof_segmentation();
                diff_static_dynamic(ImageNumber);
                publish_objects(ImageNumber);


                // if (ImageNumber==0)
                // {
                //    Objects_scene.copyTo(Objects_scene2); 
                // }
                // if (ImageNumber==1)
                // {
                //    Objects_scene2.copyTo(Objects_scene3); 
                //    Objects_scene.copyTo(Objects_scene2); 
                // }
                //     if (ImageNumber==2)
                // {
                //     Objects_scene3.copyTo(Objects_scene4); 
                //     Objects_scene2.copyTo(Objects_scene3); 
                //     Objects_scene.copyTo(Objects_scene2); 
                // }
                // if (ImageNumber>=3)
                // {
                //     diff_static_dynamic();
                //     Objects_scene3.copyTo(Objects_scene4); 
                //     Objects_scene2.copyTo(Objects_scene3); 
                //     Objects_scene.copyTo(Objects_scene2); 

                // }
                object_contour(ImageNumber);

                dTime = ((double)cv::getTickCount() - dTime)/cv::getTickFrequency();
                ROS_INFO("Time: %f seconds\n", dTime);
                ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");


                //cv::destroyAllWindows();
                // constexpr int TIME_TO_SLEEP = 500;  //////////////////////////////////////////////////////////Modificar para el tiempo
                // std::this_thread::sleep_for(std::chrono::milliseconds(TIME_TO_SLEEP));
                ImageNumber++;
                bIsProcessing=false;
            }
        }

    }



    void image_processing::segmentar()
    {

        using boost::lexical_cast;
        using std::string;
        string finish_nameRGB=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameRGB;
        // string nombreArchivo = "/home/ikerlan/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
        

        string nameFileResFinSeg= "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_application/ca_apf_scene_segmentation/scripts/folder_save/ResFinSeg";
        nameFileResFinSeg += numstring;


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



    void image_processing::ObtenerObjetosRGB()
    {
        using boost::lexical_cast;
        using std::string;
        string finish_nameRGB=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameRGB;
        // string nombreArchivo = "/home/ikerlan/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
        

        string nameFileSegRobot= "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_application/ca_apf_scene_segmentation/scripts/folder_save/SegRobot";
        nameFileSegRobot+= numstring;
        //cv::imwrite(nameFileSegRobot, SegRobot);


        cv::Mat kernel,difRobot1,difRobot2,ImageProf_Threshold,ImageProf2_Threshold,ImageProf_Threshold_grey,ImageProf2_Threshold_grey;
        // ancient
        cv::threshold(ImagenProf_U8, ImageProf_Threshold, 2, 255, cv::THRESH_BINARY);
        cv::cvtColor(ImageProf_Threshold, ImageProf_Threshold_grey, cv::COLOR_BGR2GRAY);
        cv::threshold(ImageProf_Threshold_grey, ImageProf_Threshold_grey, 2, 255, cv::THRESH_BINARY);

        // now
        cv::threshold(ImagenProf2_U8, ImageProf2_Threshold, 2, 255, cv::THRESH_BINARY);
        cv::cvtColor(ImageProf2_Threshold, ImageProf2_Threshold_grey, cv::COLOR_BGR2GRAY);
        cv::threshold(ImageProf2_Threshold_grey, ImageProf2_Threshold_grey, 2, 255, cv::THRESH_BINARY);


        // cv::imshow("ImageProf_Threshold_grey", ImageProf_Threshold_grey );
        // cv::waitKey(5000);

        //  subtract R1
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

        //  subtract R2
        cv::subtract(ResFinSeg, ImageProf2_Threshold_grey, difRobot2);
        cv::subtract(ResFinSeg, difRobot2, difRobot2);
            
        // cv::namedWindow("Diferencia de color de imagen actual",cv::WINDOW_AUTOSIZE);
        // cv::imshow("Diferencia de color de imagen actual", difRobot2 );
        // cv::waitKey(5000);

        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    
        cv::threshold(difRobot2, difRobot2, 5, 255, cv::THRESH_BINARY);

        BlackImage.copyTo(SegRobot);


        // sumar las 2
        cv::add(difRobot2, difRobot1, SegRobot);
        // cv::namedWindow("Segmentación imagen",cv::WINDOW_AUTOSIZE);
        // cv::imshow("Segmentación imagen", SegRobot);
        // cv::waitKey(1);
        //cv::imwrite(nameFileSegRobot, SegRobot);
    }


        



    void image_processing::entre_articulaciones(float x1_rect,float y1_rect,float x2_rect,float y2_rect,cv::Mat ImagenSolucion )
    {
        float rampa,x_rect,y_rect,x_rect_desp,y_rect_desp;
        bool direccion=true, direccionX=false,direccionY=false;

        rampa=((y2_rect-y1_rect)/(x2_rect-x1_rect));
        // std::cout << "rampa " <<  rampa  << '\n'; 
            

        x_rect_desp= float(x2_rect)-float(x1_rect);
        y_rect_desp= float(y2_rect)-float(y1_rect);

    
        if(x_rect_desp>10)
        {
            // printf("Derecha\n");
            // std::cout << "x_rect_desp " <<  x_rect_desp  << '\n'; 
            x_rect=x_rect_desp;
            direccion=true;
        }
        else if(x_rect_desp<-10)
        {
            // printf("Izquierda\n"); 
            
            // std::cout << "x_rect_desp " <<  x_rect_desp  << '\n';
            x_rect=x_rect_desp;  
            direccion=false;
        }
        else
        {
            // printf("No se desplaza\n");
            // std::cout << "x_rect_desp " <<  x_rect_desp  << '\n';
            x_rect=0; 
            direccionX=true;
        }


        if(y_rect_desp<-10)
        {
            // printf("Arriba\n");
            // std::cout << "y_rect_desp " <<  y_rect_desp  << '\n'; 
            y_rect=y_rect_desp;
        }
        else if(y_rect_desp>10)
        {
            // printf("Abajo\n"); 
            // std::cout << "y_rect_desp " <<  y_rect_desp  << '\n'; 
            y_rect=y_rect_desp;
        }
        else
        {
            // printf("No se desplaza\n");
            // std::cout << "y_rect_desp " <<  y_rect_desp  << '\n'; 
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


        if (direccion==true and direccionY==false and direccionX==false)
        {
            // printf("direccion==true and direccionY==false and direccionX==false\n");
            // std::cout << "x1_rect " <<  x1_rect  << '\n';
            // std::cout << "x1_rect+x_rect " <<  x1_rect+x_rect  << '\n';

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
                // std::cout << "i " <<  i  << '\n'; 
                // printf("Calcula\n");
                y_rect=rampa*(float(i)-x1_rect)+y1_rect;
                
                // printf("valor calculo\n");
                // std::cout << "y_rect " <<  y_rect  << '\n'; 
                for (size_t k=i-35; k<i+35;k++)
                {
                    
                    for (size_t n = int(y_rect)-30; n < int(y_rect+30) ; n++)
                    {
                        //  std::cout << "n " <<  n  << '\n';
                        // std::cout << "k " <<  k  << '\n'; 
                    

                        ImagenSolucion.at<uint8_t>(cv::Point(k,n))=255;
                    }
                }
            }
        }
        else if (direccion==false and direccionY==false and direccionX==false)
        {
            // printf("direccion==false and direccionY==false and direccionX==false\n");
            // std::cout << "x1_rect " <<  x1_rect  << '\n'; 

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
                // std::cout << "i " <<  i  << '\n'; 
                // printf("Calcula\n");
                y_rect=rampa*(float(i)-x1_rect)+y1_rect;
                
                // printf("valor calculo\n");
                // std::cout << "y_rect " <<  y_rect  << '\n'; 
                for (size_t k=i-35; k<i+35;k++)
                {
                    
                    for (size_t n = int(y_rect)-35; n < int(y_rect+35) ; n++)
                    {
                        // std::cout << "n " <<  n  << '\n';
                        // std::cout << "k " <<  k  << '\n'; 

                        ImagenSolucion.at<uint8_t>(cv::Point(k,n))=255;
                    }
                }
            }
        }
        else if ( direccionX==true and direccionY==false)
        {
            //printf("direccionX==true and direccionY==false\n");
            if(y1_rect-y2_rect>0)
            {
                // std::cout << "y1_rect " <<  y1_rect  << '\n';
                // std::cout << "y2_rect " <<  y2_rect << '\n'; 
                // std::cout << "x1_rect " <<  x1_rect  << '\n';
                // std::cout << "x2_rect " <<  x2_rect << '\n';
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
                        // std::cout << "n " <<  n  << '\n';
                        // std::cout << "k " <<  k  << '\n'; 
                        
                        // std::cout << "n " <<  n  << '\n';
                        // std::cout << "k " <<  k  << '\n'; 

                        ImagenSolucion.at<uint8_t>(cv::Point(k,n))=255;
                    }
                }
            }
            else
            {
                // std::cout << "y1_rect " <<  y1_rect  << '\n';
                // std::cout << "y2_rect " <<  y2_rect << '\n'; 
                // std::cout << "x1_rect " <<  x1_rect  << '\n';
                // std::cout << "x2_rect " <<  x2_rect << '\n';
                if(y1_rect-35<0)
                {
                    y1_rect=y1_rect+(35-y2_rect);
                } 

                for (size_t k=x1_rect-30; k<=x1_rect+x_rect+30;k++)
                {
                    for (size_t n = int(y1_rect)-35; n < int(y2_rect+35) ; n++)
                    {
                        // std::cout << "n " <<  n  << '\n';
                        // std::cout << "k " <<  k  << '\n'; 
                        ImagenSolucion.at<uint8_t>(cv::Point(k,n))=255;
                    }
                }
            }
        }
        else if ( direccionY==true and direccionX==false)
        {
            //printf("direccionY==true and direccionX==false\n");
            if(x1_rect-x2_rect>0)
            {
            
                // std::cout << "y1_rect " <<  y1_rect  << '\n';
                // std::cout << "y2_rect " <<  y2_rect << '\n'; 
                // std::cout << "x1_rect " <<  x1_rect  << '\n';
                // std::cout << "x2_rect " <<  x2_rect << '\n';
                if (x2_rect-20<0)
                {
                    x2_rect=x2_rect+(20-x2_rect);
                } 

                for (size_t k=x2_rect-30; k<=x1_rect+30;k++)
                {
                    for (size_t n = int(y2_rect)-20; n < int(y1_rect+20) ; n++)
                    {
                        // std::cout << "n " <<  n  << '\n';
                        // std::cout << "k " <<  k  << '\n'; 
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
                        // std::cout << "n " <<  n  << '\n';
                        // std::cout << "k " <<  k  << '\n'; 
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
        

        R2 = Eigen::Quaternionf(shoulder.getRotation().w(), shoulder.getRotation().x(), shoulder.getRotation().y(), shoulder.getRotation().z()).toRotationMatrix();
        Pc2 <<  shoulder.getOrigin().x(),
                shoulder.getOrigin().y(),
                shoulder.getOrigin().z();

        R3 = Eigen::Quaternionf(forearm.getRotation().w(), forearm.getRotation().x(), forearm.getRotation().y(), forearm.getRotation().z()).toRotationMatrix();
        // printf("R3");
        // std::cout << R3 << '\n';

        Pc3 <<  forearm.getOrigin().x(),
                forearm.getOrigin().y(),
                forearm.getOrigin().z();

        // printf("Pc3");
        // std::cout << Pc3 << '\n';

        R4 = Eigen::Quaternionf(wrist_1.getRotation().w(), wrist_1.getRotation().x(), wrist_1.getRotation().y(), wrist_1.getRotation().z()).toRotationMatrix();
        // printf("R3");
        // std::cout << R3 << '\n';

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
        // printf("shoulder_Vect");
        // std::cout << shoulder_Vect << '\n';
        // printf("forearm_Vect");
        //std::cout << forearm_Vect << '\n';
        ValueX3_real = double(shoulder_Vect.coeff(0, 0));
        ValueY3_real= double(shoulder_Vect.coeff(1, 0));
        ValueZ3_real= double(shoulder_Vect.coeff(2, 0));

        // printf("ValorX3_real");
        // std::cout << ValueX3_real << '\n';
        // printf("ValorY3_real");
        // std::cout << ValueY3_real << '\n';
        // printf("ValorZ3_real");
        // std::cout << ValueZ3_real << '\n';
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
        
        BlackImage.copyTo(Prof_No_Robot);
        BlackImage.copyTo(SegNoRobot);
        BlackImage.copyTo(Objects);
        // cv::imshow("Prof_No_Robot", Prof_No_Robot); /// antes ImagenNegro11
        // cv::waitKey(10000);


        for (size_t i = 0; i < mDepthDistance.size().width; i++)
        {
            for(size_t j=0; j < mDepthDistance.size().height; j++) 
            {
                // printf("\nvalor:");
                // std:: cout <<mDepthDistance.at<uint16_t>(cv::Point(i,j));
                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageforearm-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageforearm+70) 
                {
                    if((i>x_imageforearm and i<x_imageforearm+70 and y_imageforearm+30>j and y_imageforearm-30<j) or (i<x_imageforearm and i>x_imageforearm-70 and  y_imageforearm+30>j and y_imageforearm-30<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }
                    

                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageshoulder-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageshoulder+70) 
                {
                    if((i>x_imageshoulder and i<x_imageshoulder+70 and y_imageshoulder+50>j and y_imageshoulder-50<j) or (i<x_imageshoulder+70 and i>x_imageshoulder-70 and  y_imageshoulder+50>j and y_imageshoulder-50<j))
                    {
                       Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }


                /////////////////////// añadida
                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageshoulder_real-150 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageshoulder_real+150) 
                {
                    if((i>x_imageshoulder_real and i<x_imageshoulder_real+70 and y_imageshoulder_real+50>j and y_imageshoulder_real-50<j) or (i<x_imageshoulder_real+70 and i>x_imageshoulder_real-70 and  y_imageshoulder_real+50>j and y_imageshoulder_real-50<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }


                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageforearm_real-140 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageforearm_real+140) 
                {
                    if((i>x_imageforearm_real and i<x_imageforearm_real+70 and y_imageforearm_real+50>j and y_imageforearm_real-50<j) or (i<x_imageforearm_real+70 and i>x_imageforearm_real-70 and  y_imageforearm_real+50>j and y_imageforearm_real-50<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }


                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_1_real-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_1_real+120) 
                {
                    if((i>x_imagewrist_1_real and i<x_imagewrist_1_real+70 and y_imagewrist_1_real+50>j and y_imagewrist_1_real-50<j) or (i<x_imagewrist_1_real+70 and i>x_imagewrist_1_real-70 and  y_imagewrist_1_real+50>j and y_imagewrist_1_real-50<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }


                ////////////
                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_1-100 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_1+100) 
                {
                    if((i>x_imagewrist_1 and i<x_imagewrist_1+70 and y_imagewrist_1+30> j and y_imagewrist_1-30<j) or (i<x_imagewrist_1 and i>x_imagewrist_1-70 and  y_imagewrist_1+30>j and y_imagewrist_1-30<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;

                    }
                }

                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_2-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_2+120) 
                {
                    if((i>x_imagewrist_2 and i<x_imagewrist_2+70 and y_imagewrist_2+50>j and y_imagewrist_2-40<j) or (i<x_imagewrist_2 and i>x_imagewrist_2-70 and  y_imagewrist_2+50>j and y_imagewrist_2-40<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }

                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_3-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_3+120) 
                {
                    if((i>x_imagewrist_3 and i<x_imagewrist_3+70 and y_imagewrist_3+30>j and y_imagewrist_3-30<j) or (i<x_imagewrist_3 and i>x_imagewrist_3-70 and  y_imagewrist_3+30>j and y_imagewrist_3-30<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }

                if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagetool_center_point-150 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagetool_center_point+150) 
                {
                    if((i>x_imagetool_center_point and i<x_imagetool_center_point+70 and y_imagetool_center_point+50>j and y_imagetool_center_point-30<j) or (i<x_imagetool_center_point and i>x_imagetool_center_point-70 and  y_imagetool_center_point+50>j and y_imagetool_center_point-30<j))
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }

    
                /* Meassures between the joints */
                /// shoulder -forearm
                if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageforearm_real-130 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageshoulder_real+120) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageforearm_real+130 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageshoulder_real-120))                                                  
                {
                    if (shoulder_forearm_real.at<uint8_t>(cv::Point(i,j))==255)
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }


                /// forearm-wrist_1
                if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageforearm-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_1+100) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageforearm+120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_1-100))                                                   
                {
                    if (forearm_wrist_1.at<uint8_t>(cv::Point(i,j))==255)
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }    
                    

                /// wrist_1-wrist_2
                if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_2-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_1+120) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_2+120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_1-120))                                                   
                {
                    if (wrist_1_wrist_2.at<uint8_t>(cv::Point(i,j))==255)
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }


                /// wrist_2-wrist_3
                if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_2-140 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_3+140) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_2+140 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_3-140))                                                   
                {
                    if (wrist_2_wrist_3.at<uint8_t>(cv::Point(i,j))==255)
                    {
                        Prof_No_Robot.at<uint8_t>(cv::Point(i,j))=255;
                    }
                }


                /// wrist_3-tool_center_point
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


        using boost::lexical_cast;
        using std::string;
        string finish_nameRGB=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameRGB;
        // string nombreArchivo = "/home/ikerlan/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
        
        string nameFileProf_No_Robot= "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_application/ca_apf_scene_segmentation/scripts/folder_save/Prof_No_Robot";
        string nameFileProf_No_Robot_mor= "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_application/ca_apf_scene_segmentation/scripts/folder_save/Prof_No_Robotmor";
        nameFileProf_No_Robot+= numstring;
        nameFileProf_No_Robot_mor+= numstring;


        cv::Mat kernel;
        // cv::imshow("Quitar Robot de la imagen de profundidad", Prof_No_Robot); /// antes ImagenNegro11
        // cv::waitKey(1);
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


        for (size_t i = 0; i < mDepthDistance.size().width; i++)
        {
            for(size_t j=0; j < mDepthDistance.size().height; j++) 
            {
                if (SegNoRobot.at<uint8_t>(cv::Point(i,j))==255)
                {

                    if (mDepthDistance.at<uint16_t>(cv::Point(i,j))<*result.first-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>20 )
                    {
                        Objects.at<uint8_t>(cv::Point(i,j))=255;
                        //  z = ImagenProf.at<uint16_t>(cv::Point(i,j));
                        //             printf("Valor de pixel:");
                        //             std::cout << z << '\n';
                    }
                    else
                    {
                        //Objetos.at<uint8_t>(cv::Point(i,j))=255;

                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageBase and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageBase) 
                        {
                            // RobotFlag=true;
                            // printf("1\n");
                            // 19
                        }

                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageforearm-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageforearm+120) 
                        {
                            if((i>x_imageforearm and i<x_imageforearm+50 and y_imageforearm+30>j and y_imageforearm-30<j) or (i<x_imageforearm and i>x_imageforearm-50 and  y_imageforearm+30>j and y_imageforearm-30<j))
                            {
                                RobotFlag=true;
                                //printf("2");
                                //17cm
                            }
                        }


                        /////////////////////// añadida
                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageshoulder_real-125 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageshoulder_real+125) 
                        {
                            if((i>x_imageshoulder_real and i<x_imageshoulder_real+70 and y_imageshoulder_real+50>j and y_imageshoulder_real-50<j) or (i<x_imageshoulder_real+70 and i>x_imageshoulder_real-70 and  y_imageshoulder_real+50>j and y_imageshoulder_real-50<j))
                            {
                                RobotFlag=true;
                            }
                        }

                            
                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageshoulder-100 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageshoulder+100) 
                        {
                            if((i>x_imageshoulder and i<x_imageshoulder+60 and y_imageshoulder+30>j and y_imageshoulder-30<j) or (i<x_imageshoulder and i>x_imageshoulder-60 and  y_imageshoulder+30>j and y_imageshoulder-30<j))
                            {
                                // Si esta del todo girado 27 cm de profundidad menos
                                RobotFlag=true;
                                //printf("3\n");
                            }
                        }

                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_1-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_1+70) 
                        {
                            if((i>x_imagewrist_1 and i<x_imagewrist_1+60 and y_imagewrist_1+30> j and y_imagewrist_1-30<j) or (i<x_imagewrist_1 and i>x_imagewrist_1-60 and  y_imagewrist_1+30>j and y_imagewrist_1-30<j))
                            {
                                //18cm hasta la otra parte
                                RobotFlag=true;
                                //printf("4");
                            }
                        }

                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_2-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_2+70) 
                        {
                            if((i>x_imagewrist_2 and i<x_imagewrist_2+40 and y_imagewrist_2+30>j and y_imagewrist_2-30<j) or (i<x_imagewrist_2 and i>x_imagewrist_2-40 and  y_imagewrist_2+30>j and y_imagewrist_2-30<j))
                            {
                                RobotFlag=true;
                                //printf("5\n");
                            }
                        }

                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagewrist_3-70 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagewrist_3+70) 
                        {
                            if((i>x_imagewrist_3 and i<x_imagewrist_3+50 and y_imagewrist_3+30>j and y_imagewrist_3-30<j) or (i<x_imagewrist_3 and i>x_imagewrist_3-50 and  y_imagewrist_3+30>j and y_imagewrist_3-30<j))
                            {
                                RobotFlag=true;
                                //printf("6\n");
                            }
                        }

                        if (mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imagetool_center_point-120 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imagetool_center_point+120) 
                        {                   
                            if((i>x_imagetool_center_point and i<x_imagetool_center_point+50 and y_imagetool_center_point+50>j and y_imagetool_center_point-30<j) or (i<x_imagetool_center_point and i>x_imagetool_center_point-50 and  y_imagetool_center_point+50>j and y_imagetool_center_point-30<j))
                            {
                                RobotFlag=true;
                                //printf("7\n");
                            }
                        }

    
                        /* Meassures between singularities */
                        // shoulder -forearm
                        if ((mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageforearm_real-170 and mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageshoulder_real+170) or (mDepthDistance.at<uint16_t>(cv::Point(i,j))<z_imageforearm_real+170 and mDepthDistance.at<uint16_t>(cv::Point(i,j))>z_imageshoulder_real-170))                                                  
                        {
                            if (shoulder_forearm_real.at<uint8_t>(cv::Point(i,j))==255)
                            {
                                RobotFlag=true;
                                //printf("11\n");
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
                            //printf("28\n");
                        }
                    
                        if (RobotFlag==false)
                        {
                            Objects.at<uint8_t>(cv::Point(i,j))=255;
                            //                                                  printf("\nValor de pixel es:");
                            //                           z = ImagenProf2.at<uint16_t>(cv::Point(i,j));
                            // std::cout << z << '\n';
                    

                            //                          z = ImagenProf.at<uint16_t>(cv::Point(i,j));
                            // printf("Valor de pixel es:");
                            // std::cout << z << '\n';


                        }
                        // else
                        // {
                        //     gris2.at<uint8_t>(cv::Point(i,j))=255;
                        // }
                        RobotFlag=false;
                    }
                }
                    
            }
        }

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
        BlackImage.copyTo(Shadow_Image);
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

        //     cv::imshow("Shadow_Image", Shadow_Image);
        // cv::waitKey(10000);

        cv:: subtract(Objects,Shadow_Image,Objects);
        // cv::imshow("Objects", Objects);
        // cv::waitKey(10000);
    }

    void image_processing::Prof_background()
    {
        BlackImage.copyTo(Image_Prof_Background);
        for (size_t i = 0; i < ImageProf.size().width; i++)
        {
            for(size_t j=0; j < ImageProf.size().height; j++) 
            {
                if (ImageProf.at<uint16_t>(cv::Point(i,j))<300)
                {

                    Image_Prof_Background.at<uint8_t>(cv::Point(i,j))=255;
                }
            }
        }

        // cv::imshow("Image_Prof_Background", Image_Prof_Background);
        // cv::waitKey(10000);    
    } 


    void image_processing:: Prof_segmentation()
    {
        cv::Mat kernel;
        BlackImage.copyTo(Objects_scene);
        //    cv::imshow("Prof_No_Robot_Old", Prof_No_Robot_Old);
        // cv::waitKey(10000);
        //    cv::imshow("Prof_No_Robot", Prof_No_Robot);
        // cv::waitKey(10000);
       

        cv::subtract(Prof_No_Robot_Old,Prof_No_Robot,Prof_seg_Image);
        // cv::imshow("Prof_segmentation", Prof_seg_Image);
        // cv::waitKey(10000);


        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(20, 20));
        cv::erode(Prof_seg_Image, Prof_seg_Image, kernel);
        cv::dilate(Prof_seg_Image, Prof_seg_Image, kernel);
        // cv::imshow("Prof_seg_Image", Prof_seg_Image);
        // cv::waitKey(10000);
        cv:: add(Prof_seg_Image,Objects,Objects_scene);
        // cv::imshow("Objects_scene", Objects_scene);
        // cv::waitKey(10000);


        using boost::lexical_cast;
        using std::string;
        string finish_nameRGB=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameRGB;
                // string nombreArchivo = "/home/ikerlan/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
        string nameFileObjects_scene= "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_application/ca_apf_scene_segmentation/scripts/folder_save/Objects_scene";
    
        nameFileObjects_scene += numstring;

        //cv::imwrite(nameFileObjects_scene, Objects_scene);
    }


    // TODO: Check if this part of the code is required or it can be deleted
    // bool image_processing:: diff_static_dynamic()
    // {
    //     BlackImage.copyTo(static_object);
    //     BlackImage.copyTo(dynamic_object);
    //     cv::bitwise_and(Objects_scene, Objects_scene2, static_object, Objects_scene3); 
    //     cv::bitwise_and(static_object, static_object, static_object, Objects_scene4); 
    //     // cv::imshow("static_object", static_object);
    //     // cv::waitKey(10000);
    //     cv::subtract(Objects_scene,static_object,dynamic_object);
    //     // cv::imshow("dynamic_object", dynamic_object);
    //     // cv::waitKey(10000);
    // }


    void image_processing:: object_contour(int cont)
    {
        cv::Mat canny_Objects_scene,canny_static_object,canny_dynamic_object ;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat Objects_scene_RGB = ImageRGB2.clone();
        cv::Mat static_object_RGB = ImageRGB2.clone();
        cv::Mat dynamic_object_RGB = ImageRGB2.clone();


        using boost::lexical_cast;
        using std::string;
        string finish_nameRGB=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameRGB;
        // string nombreArchivo = "/home/ikerlan/camera_localization_ws/src/camera_localization/scripts/folder_save/Valores64.txt";
        string nameFilestatic_object_RGB= "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_application/ca_apf_scene_segmentation/scripts/folder_save/static_object_RGB";
        string nameFiledynamic_object_RGB= "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_application/ca_apf_scene_segmentation/scripts/folder_save/dynamic_object_RGB";
        nameFilestatic_object_RGB += numstring;
        nameFiledynamic_object_RGB+= numstring;


        cv::Canny( Objects_scene, canny_Objects_scene, 100, 200 );
        cv::findContours( canny_Objects_scene, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
        cv::drawContours(Objects_scene_RGB, contours, -1, cv::Scalar(0, 255, 0), 2);
        // cv::imshow("Objects_scene_RGB", Objects_scene_RGB);
        // cv::waitKey(1);


        if (cont>=3)
        {
            cv::Canny( static_object, canny_static_object, 100, 200 );
            cv::findContours( canny_static_object, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
            cv::drawContours(static_object_RGB, contours, -1, cv::Scalar(255, 0, 0), 2);
            cv::imshow("static_object_RGB", static_object_RGB);
            cv::waitKey(1);
            //cv::imwrite(nameFilestatic_object_RGB, static_object_RGB);

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
        cv::Mat image_Buffer, image_buffer_res,imageMoment;
        bool First_Flag=false;
        int Buf_num=0;


        using boost::lexical_cast;
        using std::string;
        string finish_nameProf=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameProf;
        
        string nameFileProf = "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_apf_application/ca_application/ca_apf_scene_segmentation/scripts/folder_save/static";
        nameFileProf += numstring;
        Objects_scene.copyTo(imageMoment);
        

        if(cont>=BufferSize)
        {
            // for (const cv::Mat& i :ObjectsSceneBuffer) 
            // {
            //     cv::imshow(" imagenes que se quedan", i);
            //     cv::waitKey(5000);
            //     std::cout<<Buf_num;
            //     printf("\n");
            //     Buf_num++;
            // }

            Buf_num=0;
            for (const cv::Mat& i :ObjectsSceneBuffer) 
            {
                if (First_Flag==false)
                {
                    cv::bitwise_and(imageMoment,i,image_buffer_res,imageMoment);
                    // cv::imshow("image_buffer_res 1", image_buffer_res );
                    // cv::waitKey(5000);
                    First_Flag=true;
                }
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
                // cv::imshow(" image_buffer_res for", image_buffer_res);
                // cv::waitKey(5000);
                // std::cout<<Buf_num;
                // printf("\n");              
                Buf_num++;
            }

            // cv::imshow(" image_buffer_res", image_buffer_res);
            // cv::waitKey(10000);
            ObjectsSceneBuffer.assign (ObjectsSceneBuffer.begin()+1,ObjectsSceneBuffer.end());
            ObjectsSceneBuffer.push_back(imageMoment); 
            //cv::imwrite(nameFileProf, image_buffer_res); 


            Buf_num=0;
            // for (const cv::Mat& i :ObjectsSceneBuffer) 
            //     {
            //         cv::imshow(" imagenes que se quedan para la siguiente", i);
            //         cv::waitKey(5000);
            //         std::cout<<Buf_num;
            //         printf("\n");
            //         Buf_num++;
            //     }

            
            image_buffer_res.copyTo(static_object);
            cv::subtract(Objects_scene,static_object,dynamic_object);
            // cv::imshow("dynamic_object", dynamic_object);
            // cv::waitKey(1);
            // cv::imshow("static_object", static_object);
            // cv::waitKey(1);
        }
    
        if(cont<BufferSize)
        {
            printf("guarda imagen menor de buff\n");
            ObjectsSceneBuffer.push_back(imageMoment); 
        }
    }


   
    void image_processing::publish_objects(int cont)
    {
        cv::Mat prof_image,scene_image,static_prof,dynamic_prof,scene_prof,image_static_prof,image_dynamic_prof,image_scene_prof,scene_static,scene_static_prof,copyDynamic,kernel;
        sensor_msgs::Image img_msg_scene,img_msg_static,img_msg_dynamic;
        std_msgs::Header Header; // empty header
        sensor_msgs::CameraInfo all_camera_info;

        all_camera_info=ciDepthInfo1;

        using boost::lexical_cast;
        using std::string;
        string finish_nameProf=".jpg";
        string numstring;
        numstring = lexical_cast<string>(ImageNumber);
        numstring += finish_nameProf;
        
        string nameFileStatic = "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_application/ca_apf_scene_segmentation/scripts/folder_save/static_env";
        nameFileStatic += numstring;
        string nameFileDynamic = "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_application/ca_apf_scene_segmentation/scripts/folder_save/dynamic_env";
        nameFileDynamic += numstring;
        string nameFileScene = "/home/ikerlan/ros_wss/ws_ca_apf/src/ca_application/ca_apf_scene_segmentation/scripts/folder_save/scene_env";
        nameFileScene += numstring;


        // header.seq=counter;
        Header.stamp=ros::Time::now();
        Header.frame_id = "/rs_d435_cam_color_optical_frame";
        ImageProf2.copyTo(prof_image);
        Prof_No_Robot.copyTo(scene_image);

        
        cv::threshold(scene_image, scene_image, 40, 255, cv::THRESH_BINARY_INV);
        cv::bitwise_and(prof_image,prof_image,scene_prof,scene_image);
        cv_bridge::CvImage img_bridge;
        if (cont<BufferSize)
        {
            img_bridge = cv_bridge::CvImage(Header, sensor_msgs::image_encodings::TYPE_16UC1, scene_prof);
            img_bridge.toImageMsg(img_msg_scene);

            scene_pub.publish(img_msg_scene);
            scene_pub_info.publish(all_camera_info);
            cv::convertScaleAbs(scene_prof, image_scene_prof, 0.03);
            cv::applyColorMap(image_scene_prof, image_scene_prof, cv::COLORMAP_JET);
            cv::namedWindow("scene_prof",cv::WINDOW_AUTOSIZE);
            cv::imshow("scene_prof", image_scene_prof );
            cv::waitKey(1);
        }



        if(cont>BufferSize)
        {
            // static_object
            cv::bitwise_and(prof_image,prof_image,static_prof,static_object);
            img_bridge = cv_bridge::CvImage(Header, sensor_msgs::image_encodings::TYPE_16UC1, static_prof);
            img_bridge.toImageMsg(img_msg_static);

            static_pub.publish(img_msg_static);
            static_pub_info.publish(all_camera_info);
            cv::convertScaleAbs(static_prof, image_static_prof, 0.03);
            cv::applyColorMap(image_static_prof, image_static_prof, cv::COLORMAP_JET);
            cv::namedWindow("static_prof",cv::WINDOW_AUTOSIZE);
            cv::imshow("static_prof", image_static_prof );
            cv::waitKey(1);
            //cv::imwrite(nameFileStatic, image_static_prof); 

            // dynamic_object
            cv::bitwise_and(prof_image,prof_image,dynamic_prof,dynamic_object);
            img_bridge = cv_bridge::CvImage(Header, sensor_msgs::image_encodings::TYPE_16UC1, dynamic_prof);
            img_bridge.toImageMsg(img_msg_dynamic);
            dynamic_pub.publish(img_msg_dynamic);
            dynamic_pub_info.publish(all_camera_info);
            cv::convertScaleAbs(dynamic_prof, image_dynamic_prof, 0.03);
            cv::applyColorMap(image_dynamic_prof, image_dynamic_prof, cv::COLORMAP_JET);
            cv::namedWindow("dynamic_object",cv::WINDOW_AUTOSIZE);
            cv::imshow("dynamic_object", image_dynamic_prof );
            cv::waitKey(1);
            //cv::imwrite(nameFileDynamic, image_dynamic_prof); 


            //scene
            Prof_No_Robot.copyTo(scene_image);
            dynamic_object.copyTo(copyDynamic);
            kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
            cv::dilate(copyDynamic, copyDynamic, kernel);
            cv::threshold(scene_image, scene_image, 40, 255, cv::THRESH_BINARY_INV);
            cv::subtract(scene_image,copyDynamic,scene_static);
            // cv::imshow("scene_static", scene_static );
            // cv::waitKey(1);
            cv::bitwise_and(prof_image,prof_image,scene_static_prof,scene_static);
            img_bridge = cv_bridge::CvImage(Header, sensor_msgs::image_encodings::TYPE_16UC1, scene_static_prof);
            img_bridge.toImageMsg(img_msg_scene);

            scene_pub.publish(img_msg_scene);
            scene_pub_info.publish(all_camera_info);
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
            /* TODO: Propper initilization of the processing pictures and auxiliar images */
            
            /* Initiliazation of boolean flags for image processing */
            bColorStored1 = false;
            bDepthStored1 = false;
            bIsProcessing = false;

            // Buffer for static objects
            BufferSize=10;

            /* Starting ros_publishing publisher for the Point Cloud publication */
           static_pub = _nh.advertise<sensor_msgs::Image>("/camera1/depth_workpieces/image",5);
           dynamic_pub = _nh.advertise<sensor_msgs::Image>("/camera1/depth_obstacles/image",5);
           scene_pub = _nh.advertise<sensor_msgs::Image>("/camera1/depth_scene/image",5);
           static_pub_info=_nh.advertise<sensor_msgs::CameraInfo>("/camera1/depth_workpieces/camera_info",5);
           dynamic_pub_info = _nh.advertise<sensor_msgs::CameraInfo>("/camera1/depth_obstacles/camera_info",5);
           scene_pub_info = _nh.advertise<sensor_msgs::CameraInfo>("/camera1/depth_scene/camera_info",5);
        }
        catch (...) {
            ROS_ERROR("The image processing application cannot be initialized...");
            return false;
        }
        return true;
    }


}
