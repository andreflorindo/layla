#ifndef LAYLA_DESCARTES_TESSERACT_PLANNER_H
#define LAYLA_DESCARTES_TESSERACT_PLANNER_H

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/service_client.h>
#include <layla_msgs/PoseBuilder.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// C++
#include <string>
#include <memory>
// #include <Eigen/Geometry>
// #include <jsoncpp/json/json.h>

// Opw
#include <opw_kinematics/opw_parameters.h>

// Descartes light
#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>

// Tesseract
#include <tesseract/tesseract.h>
#include <tesseract_common/macros.h>
#include <tesseract_common/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_collision_edge_evaluator.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/utils.h>
// #include <tesseract_motion_planners/hybrid/descartes_trajopt_array_planner.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <tesseract_rosutils/conversions.h>

namespace layla_motion_planner
{
const std::string POSE_BUILDER_SERVICE = "single_course";
const double SERVER_TIMEOUT = 5.0f; // seconds
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";
const std::string GET_ENVIRONMENT_CHANGES_SERVICE = "get_tesseract_changes_rviz";
const std::string MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract_rviz";
const std::string FOLLOW_JOINT_TRAJECTORY_ACTION = "joint_trajectory_action";
const int AXIAL_SYMMETRIC_MOTION = 1; // 0 - Axial Symmetric Trajectory; 1 - Fully Constraint trajectory
// const std::string FOLLOW_JOINT_TRAJECTORY_ACTION = "position_trajectory_controller/follow_joint_trajectory";

struct LayLaDescartesTesseractPlannerConfiguration
{
  std::string group_name;
  std::string tip_link;
  std::string base_link;
  std::string world_frame;
  std::vector<std::string> joint_names;
  int layer;
  int course;
};

class LayLaDescartesTesseractPlanner
{
public:
  LayLaDescartesTesseractPlanner();
  virtual ~LayLaDescartesTesseractPlanner();

  void initRos();
  std::vector<tesseract_motion_planners::Waypoint::Ptr> getCourse();
  // trajectory_msgs::JointTrajectory trajArrayToJointTrajectoryMsg(std::vector<std::string> joint_names, trajopt::TrajArray traj_array,bool use_time,ros::Duration time_increment);
  bool run();

protected:
  tesseract_motion_planners::DescartesMotionPlannerConfigD createDescartesPlannerConfig(const tesseract::Tesseract::ConstPtr &tesseract_ptr,
                                                                                        const std::string & /*manip*/,
                                                                                        const tesseract_kinematics::InverseKinematics::ConstPtr &kin,
                                                                                        const double robot_reach,
                                                                                        const tesseract_environment::EnvState::ConstPtr &current_state,
                                                                                        const std::vector<tesseract_motion_planners::Waypoint::Ptr> &waypoints,
                                                                                        bool use_collision_edge_evaluator = false);
  trajectory_msgs::JointTrajectory trajArrayToJointTrajectoryMsg(std::vector<std::string> joint_names,
                                                                 tesseract_common::TrajArray traj_array,
                                                                 bool use_time,
                                                                 ros::Duration time_increment);

private:
  LayLaDescartesTesseractPlannerConfiguration config_;
  ros::NodeHandle nh_;
  bool plotting_;
  bool rviz_;
  tesseract::Tesseract::Ptr tesseract_;     /**< @brief Tesseract Manager Class */
  ros::ServiceClient modify_env_rviz_;      /**< @brief Service for modifying tesseract environment in rviz */
  ros::ServiceClient get_env_changes_rviz_; /**< @brief Get the environment changes from rviz */
  ros::ServiceClient pose_builder_client_;
  ros::Publisher course_publisher_;
  std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> follow_joint_trajectory_client_;

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
