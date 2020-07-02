/* Author: Andre Florindo*/

/* Goal: With the cartesian trajectory execute Descartes planning algoritm (PlanPath) and 
        obtain the respective joint space trajectory (GetPath)
*/

#include <layla_descartes_moveit_planner.h>

namespace layla_motion_planning
{

void LayLaDescartesMoveitPlanner::planPath(std::vector<descartes_core::TrajectoryPtPtr> &input_traj,
                          std::vector<descartes_core::TrajectoryPtPtr> &output_path, moveit_msgs::RobotTrajectory &moveit_traj)
{
  //modifying start and end points such that are close to "seed_pose" -> Pose with joint values close to the desired start of the robot path
  std::vector<double> start_pose, end_pose;
  if (input_traj.front()->getClosestJointPose(config_.seed_pose, *robot_model_ptr_, start_pose) &&
      input_traj.back()->getClosestJointPose(config_.seed_pose, *robot_model_ptr_, end_pose))
  {
    ROS_INFO_STREAM("Setting trajectory start and end to JointTrajectoryPts");

    // Creating Start JointTrajectoryPt from start joint pose
    descartes_core::TrajectoryPtPtr start_joint_point = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::JointTrajectoryPt(start_pose));

    // Creating End JointTrajectoryPt from end joint pose
    descartes_core::TrajectoryPtPtr end_joint_point = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::JointTrajectoryPt(end_pose));

    // Modifying start and end of the trajectory, with a selected joint pose
    // Although not necessary, it reduces the time to get a solution and unexpected outcomes
    // With the algorith these will be later replaced
    input_traj[0] = start_joint_point;
    input_traj[input_traj.size() - 1] = end_joint_point;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to find closest joint pose to seed pose at the start or end of trajectory");
    exit(-1);
  }

  // Plan a robot path from the trajectory
  bool succeeded = planner_.planPath(input_traj);

  if (succeeded)
  {
    ROS_INFO_STREAM("Valid path was found");
  }
  else
  {
    ROS_ERROR_STREAM("Could not solve for a valid path");
    exit(-1);
  }

  // Retrieve the planned robot path.
  succeeded = planner_.getPath(output_path);

  if (!succeeded || output_path.empty())
  {
    ROS_ERROR_STREAM("Failed to retrieve robot path");
  }

  fromDescartesToMoveitTrajectory(output_path, moveit_traj.joint_trajectory);
  addTimeParameterizationToDescartes(moveit_traj);

  ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");

  ROS_ERROR("Solution found! Hit enter key to move robot");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void LayLaDescartesMoveitPlanner::fromDescartesToMoveitTrajectory(const std::vector<descartes_core::TrajectoryPtPtr> &input_traj,
                                                 trajectory_msgs::JointTrajectory &traj)
{
  //  // Fill out information about our trajectory
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = config_.world_frame;
  traj.joint_names = config_.joint_names;

  descartes_utilities::toRosJointPoints(*robot_model_ptr_, input_traj, VELOCITY_DESCARTES, traj.points);
  // addVel(traj);
  // addAcc(traj);
}

void LayLaDescartesMoveitPlanner::addVel(trajectory_msgs::JointTrajectory &traj) //Velocity of the joints
{
  if (traj.points.size() < 3)
    return;

  auto n_joints = traj.points.front().positions.size();

  for (auto i = 0; i < n_joints; ++i)
  {
    traj.points[0].velocities[i] = 0.0f;
    traj.points[traj.points.size() - 1].velocities[i] = 0.0f;
    for (auto j = 1; j < traj.points.size() - 1; j++)
    {
      // For each point in a given joint
      //Finite difference, first order, central. Gives the average velocity, not conservative
      double delta_theta = -traj.points[j - 1].positions[i] + traj.points[j + 1].positions[i];
      double delta_time = -traj.points[j - 1].time_from_start.toSec() + traj.points[j + 1].time_from_start.toSec();
      double v = delta_theta / delta_time;
      traj.points[j].velocities[i] = v;
    }
  }
}

void LayLaDescartesMoveitPlanner::addAcc(trajectory_msgs::JointTrajectory &traj) //Velocity of the joints
{
    if (traj.points.size() < 3)
        return;

    auto n_joints = traj.points.front().positions.size();

    for (auto i = 0; i < n_joints; ++i)
    {
        traj.points[0].accelerations[i] = 0.0f;      // <- Incorrect!!!! TODO
        traj.points[traj.points.size() - 1].accelerations[i] = 0.0f;

        for (auto j = 1; j < traj.points.size()-1; j++)
        {
            // For each point in a given joint
            //Finite difference, first order, central. Gives the average velocity, not conservative
            double delta_velocity = -traj.points[j - 1].velocities[i] + traj.points[j + 1].velocities[i];
            double delta_time = -traj.points[j - 1].time_from_start.toSec() + traj.points[j + 1].time_from_start.toSec();
            double a = delta_velocity / delta_time;
            traj.points[j].accelerations[i] = a;
        }
    }
}

void LayLaDescartesMoveitPlanner::addTimeParameterizationToDescartes(moveit_msgs::RobotTrajectory &traj)
{
  robot_trajectory::RobotTrajectory robot_trajectory(robot_model_loader_->getModel(), config_.group_name);

  for (size_t i = 0; i < traj.joint_trajectory.points.size() -1; i++)
  {
    traj.joint_trajectory.points[i].velocities.clear();
    traj.joint_trajectory.points[i].accelerations.clear();
    traj.joint_trajectory.points[i].effort.clear();
  }

  robot_trajectory.setRobotTrajectoryMsg(*kinematic_state_,traj);

  //time_parameterization_.computeTimeStamps(robot_trajectory, 0.05, 1);
  
  layla_motion_planning::ConstEESpeedTimeParameterization designed_time_parameterization;
  designed_time_parameterization.computeTimeStamps(robot_trajectory, config_.tip_link, config_.ee_speed, 1, 1);

  //layla_motion_planning::TimeOptimalTrajectoryGeneration topp(0.001,0.01);
  //topp.computeTimeStamps(robot_trajectory, 0.05, 1);

  robot_trajectory.getRobotTrajectoryMsg(traj);
}

// void LayLaDescartesMoveitPlanner::addVel(trajectory_msgs::JointTrajectory &traj) //Velocity of the joints
// {
//   if (traj.points.size() < 3)
//     return;

//   auto n_joints = traj.points.front().positions.size();

//   for (auto i = 0; i < n_joints; ++i)
//   {
//     traj.points[0].velocities[i] = 0.0f;
//     //traj.points[traj.points.size()-1].velocities[i] = 0.0f;

//     for (auto j = 1; j < traj.points.size()-1; j++)
//     {
//       // For each point in a given joint
//       //Finite difference, first order, regressive
//       double delta_theta = traj.points[j].positions[i] - traj.points[j - 1].positions[i];
//       double delta_time = traj.points[j].time_from_start.toSec() - traj.points[j - 1].time_from_start.toSec();
//       double v = delta_theta / delta_time;
//       traj.points[j].velocities[i] = v;
//     }

//   }
// }

// void LayLaDescartesMoveitPlanner::addAcc(trajectory_msgs::JointTrajectory &traj) //Velocity of the joints
// {
//   if (traj.points.size() < 3)
//     return;

//   auto n_joints = traj.points.front().positions.size();

//   for (auto i = 0; i < n_joints; ++i)
//   {
//     traj.points[0].accelerations[i] = 0.0f;
//     //traj.points[traj.points.size()-1].velocities[i] = 0.0f;

//     for (auto j = 1; j < traj.points.size(); j++)
//     {
//       // For each point in a given joint
//       //Finite difference, first order, regressive
//       double delta_velocity = traj.points[j].velocities[i] - traj.points[j - 1].velocities[i];
//       double delta_time = traj.points[j].time_from_start.toSec() - traj.points[j - 1].time_from_start.toSec();
//       double a = delta_velocity / delta_time;
//       traj.points[j].accelerations[i] = a;
//     }

//   }

} // namespace layla_motion_planning
