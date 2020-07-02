/* Author: Andre Florindo*/

/* Goal: Initialize Descartes Kinematics, RobotModel and PathPlanner objects
        Also initializes Moveit Kinematics and RobotModel, as it will be necesssary to build the RobotTrajectory object 
        for the time parameterization 
*/


#include <layla_descartes_moveit_planner.h>

namespace layla_motion_planning
{

void LayLaDescartesMoveitPlanner::initDescartes()
{
    // Instantiating a robot model
    robot_model_ptr_.reset(new descartes_moveit::IkFastMoveitStateAdapter);
    robot_model_ptr_->setCheckCollisions(true);

    if (robot_model_ptr_->initialize(ROBOT_DESCRIPTION_PARAM,
                                     config_.group_name,
                                     config_.world_frame,
                                     config_.tip_link))
    {
        ROS_INFO_STREAM("Descartes Robot Model initialized");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to initialize Robot Model");
        exit(-1);
    }

    bool succeeded = planner_.initialize(robot_model_ptr_);
    if (succeeded)
    {
        ROS_INFO_STREAM("Descartes Dense Planner initialized");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to initialize Dense Planner");
        exit(-1);
    }

    loadRobotModel();

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

void LayLaDescartesMoveitPlanner::loadRobotModel()
{
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));

    kinematic_model_ = robot_model_loader_->getModel();
    if (!kinematic_model_)
    {
        ROS_ERROR_STREAM("Failed to load robot model from robot description parameter:robot_description");
        exit(-1);
    }
    joint_model_group_ = kinematic_model_->getJointModelGroup(config_.group_name);
    kinematic_state_.reset(new moveit::core::RobotState(kinematic_model_));
}

} // namespace layla_motion_planning