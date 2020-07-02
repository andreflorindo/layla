cmake_minimum_required(VERSION 3.5.0)
project(layla_trajopt_tesseract)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tesseract_rosutils
  octomap_ros
  pcl_conversions
  layla_msgs
  control_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(trajopt REQUIRED)
find_package(tesseract REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

catkin_package(
  INCLUDE_DIRS 
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
   roscpp
   roslib
   tesseract_rosutils
   octomap_ros
   pcl_conversions
   layla_msgs
   control_msgs
   moveit_ros_planning_interface
   moveit_msgs
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

# ## Declare a Executable

add_library(problem_construction src/problem_construction.cpp)
add_library(const_ee_speed_time_parameterization src/const_ee_speed_time_parameterization.cpp)
add_library(run_trajopt src/run_trajopt.cpp)

add_executable(layla_trajopt_main src/layla_trajopt_main.cpp)

add_dependencies(layla_trajopt_main layla_msgs_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(problem_construction tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
target_link_libraries(layla_trajopt_main problem_construction ${catkin_LIBRARIES})

target_link_libraries(run_trajopt const_ee_speed_time_parameterization ${catkin_LIBRARIES})

target_link_libraries(run_trajopt tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
target_link_libraries(layla_trajopt_main run_trajopt ${catkin_LIBRARIES})

tesseract_target_compile_options(problem_construction PUBLIC)
target_include_directories(problem_construction PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(problem_construction SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

tesseract_target_compile_options(const_ee_speed_time_parameterization PUBLIC)
target_include_directories(const_ee_speed_time_parameterization PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(const_ee_speed_time_parameterization SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

tesseract_target_compile_options(run_trajopt PUBLIC)
target_include_directories(run_trajopt PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(run_trajopt SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

tesseract_target_compile_options(layla_trajopt_main PRIVATE)
target_include_directories(layla_trajopt_main PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(layla_trajopt_main SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})    

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(
  TARGETS
    const_ee_speed_time_parameterization
    problem_construction
    run_trajopt
    layla_trajopt_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )
## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)