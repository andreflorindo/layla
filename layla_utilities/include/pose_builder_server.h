/* Author: Andre Florindo*/

#ifndef POSE_BUILDER_SERVER_H
#define POSE_BUILDER_SERVER_H

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <layla_msgs/PoseBuilder.h>

// C++
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <vector>

// Eigen library
#include <eigen_conversions/eigen_msg.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

struct CourseStruct
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
};

namespace layla_motion_planning
{
const double TABLE_HEIGHT = 0.78;
const double TABLE_WIDTH = 1.2;
const double APPROACH_TABLE = 0.002; 
const double XY_EXTENSION_DISTANCE = 0.02; //meters
const double XY_RAISE_DISTANCE = 0.05; //meters
const double ANGLE_RAISE = 10; //degrees
const double Z_RAISE_DISTANCE= XY_RAISE_DISTANCE*tan(ANGLE_RAISE*M_PI/180);
const double SERVER_TIMEOUT = 5.0f; // seconds
const std::string POSE_BUILDER_SERVICE = "single_course";

struct PoseBuilderConfiguration
{
    std::string world_frame;
};

class PoseBuilder
{
public:
  PoseBuilder();
  virtual ~PoseBuilder();
  void createCourse();
  geometry_msgs::PoseArray course_poses;
  

  void initServer();
  bool serviceCallback(layla_msgs::PoseBuilder::Request &request, layla_msgs::PoseBuilder::Response &response);

protected:
  void readFileContent(std::string start_filename, CourseStruct &course);
  void introduceSmoothApproximantion(int i, CourseStruct &tangent, CourseStruct &binormal, CourseStruct &course);
  

protected:
  PoseBuilderConfiguration config_;
  ros::NodeHandle nh_;
  ros::ServiceServer pose_builder_server_;                                                                                                                                                                        

};

} 
#endif