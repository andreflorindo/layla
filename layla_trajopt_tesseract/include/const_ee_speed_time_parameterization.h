/* Author: Andre Florindo*/
#ifndef CONST_EE_VELOCITY_TIME_PARAMETERIZATION_H
#define CONST_EE_VELOCITY_TIME_PARAMETERIZATION_H

#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/JointLimits.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>

namespace layla_motion_planning
{
static const double DEFAULT_VEL_MAX = 1.0;
static const double DEFAULT_ACCEL_MAX = 2.0;
static const double ROUNDING_THRESHOLD = 0.01;
static const double ACCELERATE_NUMBER_POINTS = 10;

class ConstEESpeedTimeParameterization
{
public:
  ConstEESpeedTimeParameterization(unsigned int max_iterations = 100, double max_time_change_per_it = .01);
  ~ConstEESpeedTimeParameterization();

  bool computeTimeStamps(robot_trajectory::RobotTrajectory &trajectory, const std::string end_effector_frame,
                         const double ee_speed_request, const double max_velocity_scaling_factor = 1.0,
                         const double max_acceleration_scaling_factor = 1.0) const;

private:
  unsigned int max_iterations_;   /// @brief maximum number of iterations to find solution
  double max_time_change_per_it_; /// @brief maximum allowed time change per iteration in seconds

  void applyConstEESpeed(robot_trajectory::RobotTrajectory &rob_trajectory,
                         std::vector<double> &time_diff,
                         const std::string end_effector_frame,
                         const double ee_speed_request) const;
  
  bool checkEESpeed(robot_trajectory::RobotTrajectory &rob_trajectory,
                         std::vector<double> &time_diff,
                         const std::string end_effector_frame,
                         const double ee_speed_request) const;

  void applyVelocityConstraints(robot_trajectory::RobotTrajectory &rob_trajectory, std::vector<double> &time_diff,
                                const double max_velocity_scaling_factor) const;

  void applyAccelerationConstraints(robot_trajectory::RobotTrajectory &rob_trajectory, std::vector<double> &time_diff,
                                    const double max_acceleration_scaling_factor) const;

  double findT1(const double d1, const double d2, double t1, const double t2, const double a_max) const;
  double findT2(const double d1, const double d2, const double t1, double t2, const double a_max) const;
};
} // namespace layla_motion_planning

#endif