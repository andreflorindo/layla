cmake_minimum_required(VERSION 2.8.3)
project(layla_msgs)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  std_msgs
  trajectory_msgs
  visualization_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   EERequest.msg
   JointRequest.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   PoseBuilder.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs # Or other packages containing msgs
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS
   roscpp
   message_runtime
   geometry_msgs
   std_msgs
   trajectory_msgs
   visualization_msgs
)
