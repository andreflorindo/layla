/* Author: Andre Florindo*/

#ifndef LAYLA_DESCARTES_MOVEIT_PLANNER_H
#define LAYLA_DESCARTES_MOVEIT_PLANNER_H

// ROS
#include <ros/ros.h>

//LAYLA lib
#include <const_ee_speed_time_parameterization.h>
#include <layla_msgs/PoseBuilder.h>

// Eigen library
#include <eigen_conversions/eigen_msg.h>

// Action-Server
#include <actionlib/client/simple_action_client.h>

// MoveIt
#include <moveit_msgs/ExecuteTrajectoryAction.h> //or #include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Descartes
#include <descartes_utilities/ros_conversions.h>
#include <descartes_moveit/ikfast_moveit_state_adapter.h>
//#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
//#include <descartes_planner/sparse_planner.h>
#include <descartes_planner/dense_planner.h>

//  Time Parameterization
#include <const_ee_speed_time_parameterization.h>
#include <time_optimal_trajectory_generation.h>
// #include <moveit/trajectory_processing/iterative_time_parameterization.h>
// #include <moveit/trajectory_processing/iterative_spline_parameterization.h>
//#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace layla_motion_planning
{
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string EXECUTE_TRAJECTORY_ACTION = "execute_trajectory";
const double ORIENTATION_INCREMENT = 0.2f;
const std::string PLANNER_ID = "RRTConnect";
const std::string HOME_POSITION_NAME = "above-table";
const std::string JOINT_POSE_TOPIC = "joint_pose";
const double VELOCITY_DESCARTES = 0.1f;
const std::string POSE_BUILDER_SERVICE = "single_course";
const double SERVER_TIMEOUT = 5.0f; // seconds

struct LayLaDescartesMoveitPlannerConfiguration
{
    std::string group_name;
    std::string tip_link;
    std::string base_link;
    std::string world_frame;
    std::vector<double> seed_pose; /* Joint values close to the desired start of the robot path */
    std::vector<std::string> joint_names;
    double max_joint_speed_scaling_between_traj;
    double ee_speed;
    int layer;
    int course;
};

class LayLaDescartesMoveitPlanner
{
public:
    LayLaDescartesMoveitPlanner();
    virtual ~LayLaDescartesMoveitPlanner();

    void initRos();
    void initDescartes();
    bool getCourse(EigenSTL::vector_Isometry3d &poses);
    void generateTrajectory(EigenSTL::vector_Isometry3d &poses, std::vector<descartes_core::TrajectoryPtPtr> &input_traj);
    void planPath(std::vector<descartes_core::TrajectoryPtPtr> &input_traj,
                  std::vector<descartes_core::TrajectoryPtPtr> &output_path, moveit_msgs::RobotTrajectory &moveit_traj);
    void runPath(const std::vector<descartes_core::TrajectoryPtPtr> &path, const moveit_msgs::RobotTrajectory &moveit_traj);
    void loadRobotModel();

protected:
    void fromDescartesToMoveitTrajectory(const std::vector<descartes_core::TrajectoryPtPtr> &input_traj,
                                         trajectory_msgs::JointTrajectory &traj);
    void addVel(trajectory_msgs::JointTrajectory &traj);
    void addAcc(trajectory_msgs::JointTrajectory &traj);
    void addTimeParameterizationToDescartes(moveit_msgs::RobotTrajectory &traj);

protected:
    LayLaDescartesMoveitPlannerConfiguration config_;
    ros::NodeHandle nh_;
    std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>> moveit_run_path_client_ptr_; /* Sends a robot trajectory to moveit for execution */
    ros::ServiceClient pose_builder_client_;
    ros::Publisher course_publisher_;

    //Descartes
    descartes_core::RobotModelPtr robot_model_ptr_;
    //descartes_planner::SparsePlanner planner_;
    descartes_planner::DensePlanner planner_;

    // //PlanningScene
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    mutable robot_state::RobotStatePtr kinematic_state_;
    const robot_model::JointModelGroup *joint_model_group_;
    robot_model::RobotModelConstPtr kinematic_model_;
    // planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    // planning_pipeline::PlanningPipelinePtr planning_pipeline_;

    //Time Parameterization
    // trajectory_processing::IterativeParabolicTimeParameterization time_parameterization_;
    // trajectory_processing::IterativeSplineParameterization time_parameterization_;
    // trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization_;
};

} // namespace layla_motion_planning
#endif