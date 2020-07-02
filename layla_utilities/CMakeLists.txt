cmake_minimum_required(VERSION 2.8.3)
project(layla_utilities)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  eigen_conversions
  layla_msgs
  tf
  moveit_ros_planning_interface
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS 
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
   roscpp
   rospy
   roslib
   eigen_conversions
   layla_msgs
   tf
   moveit_ros_planning_interface
)

catkin_install_python(PROGRAMS src/pose_builder.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#  )

## Add cmake target dependencies of the library
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(pose_builder src/pose_builder.cpp)
add_executable(course_display src/course_display.cpp)
add_executable(ee_cartesian_pose_publisher src/ee_cartesian_pose_publisher.cpp)
add_executable(joint_request_publisher src/joint_request_publisher.cpp)
add_executable(ee_request_publisher src/ee_request_publisher.cpp)
add_executable(ee_state_publisher src/ee_state_publisher.cpp)

## Add cmake target dependencies of the executable
add_dependencies(pose_builder layla_msgs_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(course_display layla_msgs_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ee_cartesian_pose_publisher layla_msgs_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(joint_request_publisher layla_msgs_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ee_request_publisher layla_msgs_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ee_state_publisher layla_msgs_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(pose_builder ${catkin_LIBRARIES})
target_link_libraries(course_display ${catkin_LIBRARIES})
target_link_libraries(ee_cartesian_pose_publisher ${catkin_LIBRARIES})
target_link_libraries(joint_request_publisher ${catkin_LIBRARIES})
target_link_libraries(ee_request_publisher ${catkin_LIBRARIES})
target_link_libraries(ee_state_publisher ${catkin_LIBRARIES})