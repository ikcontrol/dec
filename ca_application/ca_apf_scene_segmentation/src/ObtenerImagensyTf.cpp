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
    ros::init(argc, argv, "image_processing_node");
    ros::NodeHandle nh;
    /* Instantiation of the ROS spinner to attend the call of ROS functions */
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ROS_INFO("The image proccesing has been started...");
    ros::Duration(35).sleep();           //35 //// TODO: Hablar de cómo hacer esta espera elegante
    pic_handling::image_processing imApp(nh);
    ROS_INFO("inicializacion fina");


    
    

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









    



  
