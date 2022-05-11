cmake_minimum_required(VERSION 3.0.2)
project(ca_apf_application)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    apf_msgs
    sensor_msgs
    apf_algorithms
    tf2
    tf2_ros
    tf2_eigen
    image_transport
    cv_bridge
    ur_interface_handler
    apf_controller_msgs
    pcl_conversions
    pcl_ros
    robotiq_gripper_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    control_msgs
    moveit_core
    pluginlib
    moveit_switching_controller_manager
)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    apf_msgs
    sensor_msgs
    apf_algorithms
    tf2
    tf2_ros
    tf2_eigen
    image_transport
    cv_bridge
    ur_interface_handler
    apf_controller_msgs
    pcl_conversions
    pcl_ros
    robotiq_gripper_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    control_msgs
    moveit_core
    pluginlib
    moveit_switching_controller_manager
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)

# Project library
add_library(${PROJECT_NAME}
 src/helper/robotiq_gripper_helper.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install (TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


# Obstacle publiser
add_executable(apf_obstacle_publisher_node
  src/flacco_apf_depth_obstacle_publisher_node.cpp
  src/flacco_apf_depth_obstacle_publisher.cpp
)
add_dependencies(apf_obstacle_publisher_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(apf_obstacle_publisher_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# EE apf recorder
add_executable(apf_ee_recorder
  src/apf_ee_recorder_node.cpp
)
add_dependencies(apf_ee_recorder
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(apf_ee_recorder
  ${catkin_LIBRARIES}
)

install(PROGRAMS scripts/launch_gl_compat.bash DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
