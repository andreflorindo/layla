cmake_minimum_required(VERSION 3.5.0)
project(layla_descartes_tesseract)

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
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(descartes_light REQUIRED)
find_package(descartes_samplers REQUIRED)
find_package(descartes_opw REQUIRED)
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

add_library(descartes_problem_construction src/descartes_problem_construction.cpp)
add_library(run_descartes src/run_descartes.cpp)

add_executable(layla_descartes_tesseract_main src/layla_descartes_tesseract_main.cpp)

add_dependencies(layla_descartes_tesseract_main layla_msgs_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(descartes_problem_construction tesseract::tesseract tesseract::tesseract_motion_planners_descartes tesseract::tesseract_kinematics_opw ${catkin_LIBRARIES})
target_link_libraries(layla_descartes_tesseract_main descartes_problem_construction ${catkin_LIBRARIES})

target_link_libraries(run_descartes tesseract::tesseract tesseract::tesseract_motion_planners_descartes ${catkin_LIBRARIES})
target_link_libraries(layla_descartes_tesseract_main run_descartes ${catkin_LIBRARIES})

tesseract_target_compile_options(descartes_problem_construction PUBLIC)
target_include_directories(descartes_problem_construction PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(descartes_problem_construction SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

tesseract_target_compile_options(run_descartes PUBLIC)
target_include_directories(run_descartes PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(run_descartes SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

tesseract_target_compile_options(layla_descartes_tesseract_main PRIVATE)
target_include_directories(layla_descartes_tesseract_main PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(layla_descartes_tesseract_main SYSTEM PRIVATE
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
    descartes_problem_construction
    run_descartes
    layla_descartes_tesseract_main
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_myworkcell_core.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)