/*
 * Copyright 2023 Ikerlan S. COOP.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * Date: 26/04/2021
 * 
 * Author: Diego Rodríguez
 * 
 * Description:
 * An auxiliar node for developing vision algorithm based on the green obstacles to simulate operators in the robot's surroundings. The main target of this node is to
 * publish green obstacles in the robot scene, so ROS can recognize them by the aforementioned algorithms.
 * 
*/

//////////////////////////////////////////////////////////////
////////////////////// INCLUDE ZONE //////////////////////////
//////////////////////////////////////////////////////////////
// #include "../include/simu/obstacle_spawner_class.h"     // Self-Defined: application class

#include <ros/ros.h>                                    // ROS Librarry:

#include <math.h>                                       // System Library:
#include <stdio.h>                                      // System Library: 

#include <gazebo_msgs/SpawnModel.h>                     // Gazebo Library: for spawning self-described models into gazebo space 

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>        // TF2 Library: for publishing TF2 type messages

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
    ros::init(argc, argv, "gazebo_obstacle_spawner");
    ros::NodeHandle nh;

    /* Instantiation of the ROS spinner to attend the call of ROS functions */
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ROS_INFO("Gazebo obstacle spawner node started...");
    
    /* Local variable declaration */
    std::string GAZEBO_SPAWN_SERVICE = "/gazebo/spawn_urdf_model";
    ros::service::waitForService(GAZEBO_SPAWN_SERVICE, -1);
    ROS_INFO("Trying to spawn robot green obstacle...");
    
    /* ROS nodes subscribers and publishers declarations */
    ros::ServiceClient scSpawn = nh.serviceClient<gazebo_msgs::SpawnModel>(GAZEBO_SPAWN_SERVICE);

    /* Initialization and one time execution program */
    // Load parameters
    std::string urdf;
    if (!nh.getParam("/obstacle_spawner_node/obstacle_description", urdf))
    {
        ROS_ERROR("Could not find a model to load in param '~obstacle_description'");
        return 1;
    }

    std::vector<double> rpy;
    if (!nh.getParam("/obstacle_spawner_node/orientation", rpy))
    {
        ROS_ERROR("Could not find a parameter '~orientation'");
        return 1;
    }
    if (rpy.size() != 3)
    {
        ROS_ERROR("'~orientation' should have 3 elements");
        return 1;
    }

    std::vector<double> position;
    if (!nh.getParam("/obstacle_spawner_node/position", position))
    {
        ROS_ERROR("Could not find a parameter '~position'");
        return 1;
    }
    if (position.size() != 3)
    {
        ROS_ERROR("'~position' should have 3 elements");
        return 1;
    }

    // Create service request
    gazebo_msgs::SpawnModel srv;
    srv.request.model_xml = urdf;
    srv.request.model_name = "test_obstacle";
    srv.request.robot_namespace = "";
    srv.request.reference_frame = "world";

    geometry_msgs::Pose pose;

    tf2::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    pose.orientation = tf2::toMsg(q);

    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];

    srv.request.initial_pose = pose;

    // Call the service
    ROS_INFO("Requesting obstacle spawn at [p: %f %f %f, rpy: %f %f %f]", pose.position.x, pose.position.y, pose.position.z, rpy[0], rpy[1], rpy[2]);
    scSpawn.call(srv);

    // Display service call result
    if (srv.response.success)
    {
        ROS_INFO("Object spawned successfully");
        return 0;
    }
    else
    {
        ROS_ERROR_STREAM("Object was not spawned properly: " << srv.response.status_message);
        return 1;
    }

}
