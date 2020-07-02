/* Author: Andre Florindo*/

/* Goal: Creates a RobotTrajectory with Descartes Trajectory and executes path
*/

#include <layla_descartes_moveit_planner.h>

namespace layla_motion_planning
{

void LayLaDescartesMoveitPlanner::runPath(const std::vector<descartes_core::TrajectoryPtPtr> &path, const moveit_msgs::RobotTrajectory &moveit_traj)
{

  // moving to "above-table" configuration
  // creating move group to move the arm in free space
  moveit::planning_interface::MoveGroupInterface move_group(config_.group_name);
  move_group.setPlannerId(PLANNER_ID); //RRTConnect
  move_group.setPlanningTime(10.0f);
  move_group.setMaxVelocityScalingFactor(config_.max_joint_speed_scaling_between_traj);

  // setting above-table position as target
  if (!move_group.setNamedTarget(HOME_POSITION_NAME))
  {
    ROS_ERROR_STREAM("Failed to set home '" << HOME_POSITION_NAME << "' position");
    exit(-1);
  }

  moveit_msgs::MoveItErrorCodes result = move_group.move();
  if (result.val != result.SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to move to " << HOME_POSITION_NAME << " position");
    exit(-1);
  }
  else
  {
    ROS_INFO_STREAM("Robot reached home position");
  }

  // creating goal joint pose to start of the path
  std::vector<double> seed_pose(robot_model_ptr_->getDOF());
  std::vector<double> start_pose;

  descartes_core::TrajectoryPtPtr first_point_ptr = path[0];
  first_point_ptr->getNominalJointPose(seed_pose, *robot_model_ptr_, start_pose);

  move_group.setJointValueTarget(start_pose);

  result = move_group.move();
  if (result.val != result.SUCCESS)
  {
    ROS_ERROR_STREAM("Move to start joint pose failed");
    exit(-1);
  }
  else
  {
    ROS_INFO_STREAM("Robot reached start position");
  }

  // Send Descartes trajectory
  
  moveit_msgs::ExecuteTrajectoryGoal goal;
  goal.trajectory = moveit_traj;

  ROS_INFO_STREAM("Robot path sent for execution");
  if (moveit_run_path_client_ptr_->sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO_STREAM("Robot path execution completed");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to run robot path with error " << *moveit_run_path_client_ptr_->getResult());
    exit(-1);
  }

  ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}




} // namespace layla_motion_planning
