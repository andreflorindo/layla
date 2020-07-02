#ifndef LAYLA_TRAJOPT_PLANNER_H
#define LAYLA_TRAJOPT_PLANNER_H

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/service_client.h>
#include <layla_msgs/PoseBuilder.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// C++
#include <fstream>
#include <string>
#include <memory>
#include <Eigen/Geometry>
// #include <jsoncpp/json/json.h>

// Tesseract
#include <tesseract/tesseract.h>
#include <tesseract_common/macros.h>
#include <tesseract_common/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <tesseract_rosutils/conversions.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

// Trajopt
#include <trajopt/file_write_callback.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

// Moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <const_ee_speed_time_parameterization.h>
//#include <moveit/trajectory_processing/iterative_time_parameterization.h>
//#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
//#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace layla_motion_planner
{
const std::string POSE_BUILDER_SERVICE = "single_course";
const double SERVER_TIMEOUT = 5.0f; // seconds
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";
const std::string GET_ENVIRONMENT_CHANGES_SERVICE = "get_tesseract_changes_rviz";
const std::string MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract_rviz";
const std::string FOLLOW_JOINT_TRAJECTORY_ACTION = "joint_trajectory_action";
// const std::string FOLLOW_JOINT_TRAJECTORY_ACTION = "position_trajectory_controller/follow_joint_trajectory";
const std::string PLANNER_ID = "RRTConnect";
const std::string HOME_POSITION_NAME = "above-table";

struct LayLaTrajoptPlannerConfiguration
{
  std::string group_name;
  std::string tip_link;
  std::string base_link;
  std::string world_frame;
  std::vector<std::string> joint_names;
  int layer;
  int course;
  double distance_waypoints;
  double ee_speed;
  double max_velocity_scaling;
};

class LayLaTrajoptPlanner
{
public:
  LayLaTrajoptPlanner();
  virtual ~LayLaTrajoptPlanner();

  void initRos();
  tesseract_common::VectorIsometry3d getCourse();
  trajectory_msgs::JointTrajectory trajArrayToJointTrajectoryMsg(std::vector<std::string> joint_names, trajopt::TrajArray traj_array, bool use_time, ros::Duration time_increment);
  bool run();
  void loadRobotModel();

protected:
  trajopt::ProblemConstructionInfo trajoptPCI();
  tesseract_common::TrajArray readInitTraj(std::string start_filename);
  void addVel(trajectory_msgs::JointTrajectory &traj);
  void addAcc(trajectory_msgs::JointTrajectory &traj);
  void executeStartMotionwithMoveit(trajopt::TrajArray poses_matrix);
  void addTimeParameterizationToTrajopt(moveit_msgs::RobotTrajectory &traj);

private:
  LayLaTrajoptPlannerConfiguration config_;
  ros::NodeHandle nh_;
  bool plotting_;
  bool rviz_;
  tesseract::Tesseract::Ptr tesseract_;     /**< @brief Tesseract Manager Class */
  ros::ServiceClient modify_env_rviz_;      /**< @brief Service for modifying tesseract environment in rviz */
  ros::ServiceClient get_env_changes_rviz_; /**< @brief Get the environment changes from rviz */
  ros::ServiceClient pose_builder_client_;
  ros::Publisher course_publisher_;
  std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> follow_joint_trajectory_client_;

  // //PlanningScene
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  mutable robot_state::RobotStatePtr kinematic_state_;
  const robot_model::JointModelGroup *joint_model_group_;
  robot_model::RobotModelConstPtr kinematic_model_;


  // Time Parameterization
  //trajectory_processing::IterativeParabolicTimeParameterization time_parameterization_;
  //trajectory_processing::IterativeSplineParameterization time_parameterization_;
  //trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization_;

  /**
   * @brief Check rviz and make sure the rviz environment revision number is zero.
   * @return True if revision number is zero, otherwise false.
   */
  bool checkRviz()
  {
    // Get the current state of the environment.
    // Usually you would not be getting environment state from rviz
    // this is just an example. You would be gettting it from the
    // environment_monitor node. Need to update examples to launch
    // environment_monitor node.
    get_env_changes_rviz_.waitForExistence();
    tesseract_msgs::GetEnvironmentChanges env_changes;
    env_changes.request.revision = 0;
    if (get_env_changes_rviz_.call(env_changes))
    {
      ROS_INFO("Retrieve current environment changes!");
    }
    else
    {
      ROS_ERROR("Failed to retrieve current environment changes!");
      return false;
    }

    // There should not be any changes but check
    if (env_changes.response.revision != 0)
    {
      ROS_ERROR("The environment has changed externally!");
      return false;
    }
    return true;
  }

  /**
   * @brief Send RViz the latest number of commands
   * @param n The past revision number
   * @return True if successful otherwise false
   */
  bool sendRvizChanges(unsigned long past_revision)
  {
    modify_env_rviz_.waitForExistence();
    tesseract_msgs::ModifyEnvironment update_env;
    update_env.request.id = tesseract_->getEnvironment()->getName();
    update_env.request.revision = past_revision;
    if (!tesseract_rosutils::toMsg(update_env.request.commands,
                                   tesseract_->getEnvironment()->getCommandHistory(),
                                   update_env.request.revision))
    {
      ROS_ERROR("Failed to generate commands to update rviz environment!");
      return false;
    }

    if (modify_env_rviz_.call(update_env))
    {
      ROS_INFO("RViz environment Updated!");
    }
    else
    {
      ROS_INFO("Failed to update rviz environment");
      return false;
    }

    return true;
  }
};

} // namespace layla_motion_planner

#endif
