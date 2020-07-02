cmake_minimum_required(VERSION 2.8.3)
project(layla_ompl_moveit)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_msgs
  moveit_ros_planning_interface
  eigen_conversions
  layla_msgs
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
   moveit_msgs
   moveit_ros_planning_interface
   eigen_conversions
   layla_msgs
#  DEPENDS system_lib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/const_ee_speed_time_parameterization.cpp
   src/ompl_planner_construction.cpp
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_main src/layla_ompl_moveit_main.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} layla_msgs_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_main layla_msgs_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME} ${catkin_LIBRARIES})

