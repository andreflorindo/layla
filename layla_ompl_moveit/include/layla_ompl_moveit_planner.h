#ifndef LAYLA_OMPL_MOVEIT_PLANNER_H
#define LAYLA_OMPL_MOVEIT_PLANNER_H

// ROS
#include <ros/ros.h>

//LAYLA lib
#include <const_ee_speed_time_parameterization.h>
#include <layla_msgs/PoseBuilder.h>


// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>

// 
// #include <moveit/robot_state/conversions.h>
// #include <moveit/planning_pipeline/planning_pipeline.h>
// #include <moveit/planning_interface/planning_interface.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <moveit/kinematic_constraints/utils.h>
// #include <moveit_msgs/PlanningScene.h>

// Eigen library
// #include <eigen_conversions/eigen_msg.h>

// Action-Server
// #include <actionlib/client/simple_action_client.h>

// MoveIt
// #include <moveit_msgs/ExecuteTrajectoryAction.h> //or #include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>


namespace layla_motion_planning
{
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string POSE_BUILDER_SERVICE = "single_course";
const double SERVER_TIMEOUT = 5.0f; // seconds
const std::string HOME_POSITION_NAME = "above-table";

struct LayLaOMPLMoveitPlannerConfiguration
{
    std::string group_name;
    std::string tip_link;
    std::string base_link;
    std::string world_frame;
    std::vector<double> seed_pose; /* Joint values close to the desired start of the robot path */
    std::vector<std::string> joint_names;
    std::string planner_id;
    double max_joint_speed_scaling_between_traj;
    double ee_speed;
    double distance_waypoints;
    int layer;
    int course;
};

class LayLaOMPLMoveitPlanner
{
public:
    void initOmpl();
    void getCourse(std::vector<geometry_msgs::Pose> &poses);
    void createMotionPlanRequest(std::vector<geometry_msgs::Pose> &poses);
    

protected:
    void loadRobotModel();
    void addTimeParameterizationToOmpl(moveit_msgs::RobotTrajectory &traj, double ee_speed, double max_speed_scaling);

protected:
    LayLaOMPLMoveitPlannerConfiguration config_;
    ros::NodeHandle nh_;
    ros::ServiceClient pose_builder_client_;
    ros::Publisher course_publisher_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    mutable robot_state::RobotStatePtr kinematic_state_;
    const robot_model::JointModelGroup *joint_model_group_;
    robot_model::RobotModelConstPtr kinematic_model_;
    // planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    // planning_pipeline::PlanningPipelinePtr planning_pipeline_;
};

} // namespace layla_motion_planning

#endif
