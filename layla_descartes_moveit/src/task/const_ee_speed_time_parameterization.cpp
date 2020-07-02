/* Author: Ken Anderson, Andre Florindo*/

/* Goal: Moveit IPTP with a small addition, a cartesian speed can be requested. 
        First computes the minimum time between waypoints to get the requested cartesian speed
        Only then checks if the max velocity and max acceleration of the joints are acceptable, if not replaces them
*/

#include <layla_descartes_moveit_planner.h>

namespace layla_motion_planning
{

ConstEESpeedTimeParameterization::ConstEESpeedTimeParameterization(unsigned int max_iterations,
                                                                   double max_time_change_per_it)
    : max_iterations_(max_iterations), max_time_change_per_it_(max_time_change_per_it)
{
}

ConstEESpeedTimeParameterization::~ConstEESpeedTimeParameterization() = default;

// Applies velocity
void ConstEESpeedTimeParameterization::applyVelocityConstraints(robot_trajectory::RobotTrajectory &rob_trajectory,
                                                                std::vector<double> &time_diff,
                                                                const double max_velocity_scaling_factor) const
{
    const robot_model::JointModelGroup *group = rob_trajectory.getGroup();
    const std::vector<std::string> &vars = group->getVariableNames();
    const std::vector<int> &idx = group->getVariableIndexList();
    const robot_model::RobotModel &rmodel = group->getParentModel();
    const int num_points = rob_trajectory.getWayPointCount();

    double velocity_scaling_factor = 1.0;

    if (max_velocity_scaling_factor > 0.0 && max_velocity_scaling_factor <= 1.0)
        velocity_scaling_factor = max_velocity_scaling_factor;
    else if (max_velocity_scaling_factor == 0.0)
        ROS_DEBUG_NAMED("trajectory_processing.iterative_time_parameterization",
                        "A max_velocity_scaling_factor of 0.0 was specified, defaulting to %f instead.",
                        velocity_scaling_factor);
    else
        ROS_WARN_NAMED("trajectory_processing.iterative_time_parameterization",
                       "Invalid max_velocity_scaling_factor %f specified, defaulting to %f instead.",
                       max_velocity_scaling_factor, velocity_scaling_factor);

    for (int i = 0; i < num_points - 1; ++i)
    {
        const robot_state::RobotStatePtr &curr_waypoint = rob_trajectory.getWayPointPtr(i);
        const robot_state::RobotStatePtr &next_waypoint = rob_trajectory.getWayPointPtr(i + 1);

        for (std::size_t j = 0; j < vars.size(); ++j)
        {
            double v_max = DEFAULT_VEL_MAX;
            const robot_model::VariableBounds &b = rmodel.getVariableBounds(vars[j]);
            if (b.velocity_bounded_)
                v_max =
                    std::min(fabs(b.max_velocity_ * velocity_scaling_factor), fabs(b.min_velocity_ * velocity_scaling_factor));
            const double dq1 = curr_waypoint->getVariablePosition(idx[j]);
            const double dq2 = next_waypoint->getVariablePosition(idx[j]);
            const double t_min = std::abs(dq2 - dq1) / v_max;
            if (t_min > time_diff[i])
                time_diff[i] = t_min;
        }
    }
}

// Iteratively expand dt1 interval by a constant factor until within acceleration constraint
// In the future we may want to solve to quadratic equation to get the exact timing interval.
// To do this, use the CubicTrajectory::quadSolve() function in cubic_trajectory.h
double ConstEESpeedTimeParameterization::findT1(const double dq1, const double dq2, double dt1, const double dt2,
                                                const double a_max) const
{
    const double mult_factor = 1.01;
    double v1 = (dq1) / dt1;
    double v2 = (dq2) / dt2;
    double a = 2.0 * (v2 - v1) / (dt1 + dt2);

    while (std::abs(a) > a_max)
    {
        v1 = (dq1) / dt1;
        v2 = (dq2) / dt2;
        a = 2.0 * (v2 - v1) / (dt1 + dt2);
        dt1 *= mult_factor;
    }

    return dt1;
}

double ConstEESpeedTimeParameterization::findT2(const double dq1, const double dq2, const double dt1, double dt2,
                                                const double a_max) const
{
    const double mult_factor = 1.01;
    double v1 = (dq1) / dt1;
    double v2 = (dq2) / dt2;
    double a = 2.0 * (v2 - v1) / (dt1 + dt2);

    while (std::abs(a) > a_max)
    {
        v1 = (dq1) / dt1;
        v2 = (dq2) / dt2;
        a = 2.0 * (v2 - v1) / (dt1 + dt2);
        dt2 *= mult_factor;
    }

    return dt2;
}

// Applies Acceleration constraints
void ConstEESpeedTimeParameterization::applyAccelerationConstraints(
    robot_trajectory::RobotTrajectory &rob_trajectory, std::vector<double> &time_diff,
    const double max_acceleration_scaling_factor) const
{
    robot_state::RobotStatePtr prev_waypoint;
    robot_state::RobotStatePtr curr_waypoint;
    robot_state::RobotStatePtr next_waypoint;

    const robot_model::JointModelGroup *group = rob_trajectory.getGroup();
    const std::vector<std::string> &vars = group->getVariableNames();
    const std::vector<int> &idx = group->getVariableIndexList();
    const robot_model::RobotModel &rmodel = group->getParentModel();

    const int num_points = rob_trajectory.getWayPointCount();
    const unsigned int num_joints = group->getVariableCount();
    int num_updates = 0;
    int iteration = 0;
    bool backwards = false;
    double q1;
    double q2;
    double q3;
    double dt1;
    double dt2;
    double v1;
    double v2;
    double a;

    double acceleration_scaling_factor = 1.0;

    if (max_acceleration_scaling_factor > 0.0 && max_acceleration_scaling_factor <= 1.0)
        acceleration_scaling_factor = max_acceleration_scaling_factor;
    else if (max_acceleration_scaling_factor == 0.0)
        ROS_DEBUG_NAMED("trajectory_processing.iterative_time_parameterization",
                        "A max_acceleration_scaling_factor of 0.0 was specified, defaulting to %f instead.",
                        acceleration_scaling_factor);
    else
        ROS_WARN_NAMED("trajectory_processing.iterative_time_parameterization",
                       "Invalid max_acceleration_scaling_factor %f specified, defaulting to %f instead.",
                       max_acceleration_scaling_factor, acceleration_scaling_factor);

    do
    {
        num_updates = 0;
        iteration++;

        // In this case we iterate through the joints on the outer loop.
        // This is so that any time interval increases have a chance to get propogated through the trajectory
        for (unsigned int j = 0; j < num_joints; ++j)
        {
            // Loop forwards, then backwards
            for (int count = 0; count < 2; ++count)
            {
                for (int i = 0; i < num_points - 1; ++i)
                {
                    int index = backwards ? (num_points - 1) - i : i;

                    curr_waypoint = rob_trajectory.getWayPointPtr(index);

                    if (index > 0)
                        prev_waypoint = rob_trajectory.getWayPointPtr(index - 1);

                    if (index < num_points - 1)
                        next_waypoint = rob_trajectory.getWayPointPtr(index + 1);

                    // Get acceleration limits
                    double a_max = DEFAULT_ACCEL_MAX;
                    const robot_model::VariableBounds &b = rmodel.getVariableBounds(vars[j]);
                    if (b.acceleration_bounded_)
                        a_max = std::min(fabs(b.max_acceleration_ * acceleration_scaling_factor),
                                         fabs(b.min_acceleration_ * acceleration_scaling_factor));

                    if (index == 0)
                    {
                        // First point
                        q1 = next_waypoint->getVariablePosition(idx[j]);
                        q2 = curr_waypoint->getVariablePosition(idx[j]);
                        q3 = next_waypoint->getVariablePosition(idx[j]);

                        dt1 = dt2 = time_diff[index];
                        assert(!backwards);
                    }
                    else if (index < num_points - 1)
                    {
                        // middle points
                        q1 = prev_waypoint->getVariablePosition(idx[j]);
                        q2 = curr_waypoint->getVariablePosition(idx[j]);
                        q3 = next_waypoint->getVariablePosition(idx[j]);

                        dt1 = time_diff[index - 1];
                        dt2 = time_diff[index];
                    }
                    else
                    {
                        // last point - careful, there are only numpoints-1 time intervals
                        q1 = prev_waypoint->getVariablePosition(idx[j]);
                        q2 = curr_waypoint->getVariablePosition(idx[j]);
                        q3 = prev_waypoint->getVariablePosition(idx[j]);

                        dt1 = dt2 = time_diff[index - 1];
                        assert(backwards);
                    }

                    if (dt1 == 0.0 || dt2 == 0.0)
                    {
                        v1 = 0.0;
                        v2 = 0.0;
                        a = 0.0;
                    }
                    else
                    {
                        bool start_velocity = false;
                        if (index == 0)
                        {
                            if (curr_waypoint->hasVelocities())
                            {
                                start_velocity = true;
                                v1 = curr_waypoint->getVariableVelocity(idx[j]);
                            }
                        }
                        v1 = start_velocity ? v1 : (q2 - q1) / dt1;
                        v2 = (q3 - q2) / dt2;
                        a = 2.0 * (v2 - v1) / (dt1 + dt2);
                    }

                    if (fabs(a) > a_max + ROUNDING_THRESHOLD)
                    {
                        if (!backwards)
                        {
                            dt2 = std::min(dt2 + max_time_change_per_it_, findT2(q2 - q1, q3 - q2, dt1, dt2, a_max));
                            time_diff[index] = dt2;
                        }
                        else
                        {
                            dt1 = std::min(dt1 + max_time_change_per_it_, findT1(q2 - q1, q3 - q2, dt1, dt2, a_max));
                            time_diff[index - 1] = dt1;
                        }
                        num_updates++;

                        if (dt1 == 0.0 || dt2 == 0.0)
                        {
                            v1 = 0.0;
                            v2 = 0.0;
                            a = 0.0;
                        }
                        else
                        {
                            v1 = (q2 - q1) / dt1;
                            v2 = (q3 - q2) / dt2;
                            a = 2 * (v2 - v1) / (dt1 + dt2);
                        }
                    }
                }
                backwards = !backwards;
            }
        }
        // ROS_DEBUG_NAMED("trajectory_processing.iterative_time_parameterization", "applyAcceleration: num_updates=%i",
        // num_updates);
    } while (num_updates > 0 && iteration < static_cast<int>(max_iterations_));
}

namespace
{
// Takes the time differences, and updates the timestamps, velocities and accelerations
// in the trajectory.
void updateTrajectory(robot_trajectory::RobotTrajectory &rob_trajectory, const std::vector<double> &time_diff)
{
    // Error check
    if (time_diff.empty())
        return;

    double time_sum = 0.0;

    robot_state::RobotStatePtr prev_waypoint;
    robot_state::RobotStatePtr curr_waypoint;
    robot_state::RobotStatePtr next_waypoint;

    const robot_model::JointModelGroup *group = rob_trajectory.getGroup();
    const std::vector<std::string> &vars = group->getVariableNames();
    const std::vector<int> &idx = group->getVariableIndexList();

    int num_points = rob_trajectory.getWayPointCount();

    rob_trajectory.setWayPointDurationFromPrevious(0, time_sum);

    // Times
    for (int i = 1; i < num_points; ++i)
        // Update the time between the waypoints in the robot_trajectory.
        rob_trajectory.setWayPointDurationFromPrevious(i, time_diff[i - 1]);

    // Return if there is only one point in the trajectory!
    if (num_points <= 1)
        return;

    // Accelerations
    for (int i = 0; i < num_points; ++i)
    {
        curr_waypoint = rob_trajectory.getWayPointPtr(i);

        if (i > 0)
            prev_waypoint = rob_trajectory.getWayPointPtr(i - 1);

        if (i < num_points - 1)
            next_waypoint = rob_trajectory.getWayPointPtr(i + 1);

        for (std::size_t j = 0; j < vars.size(); ++j)
        {
            double q1;
            double q2;
            double q3;
            double dt1;
            double dt2;

            if (i == 0)
            {
                // First point
                q1 = next_waypoint->getVariablePosition(idx[j]);
                q2 = curr_waypoint->getVariablePosition(idx[j]);
                q3 = q1;

                dt1 = dt2 = time_diff[i];
            }
            else if (i < num_points - 1)
            {
                // middle points
                q1 = prev_waypoint->getVariablePosition(idx[j]);
                q2 = curr_waypoint->getVariablePosition(idx[j]);
                q3 = next_waypoint->getVariablePosition(idx[j]);

                dt1 = time_diff[i - 1];
                dt2 = time_diff[i];
            }
            else
            {
                // last point
                q1 = prev_waypoint->getVariablePosition(idx[j]);
                q2 = curr_waypoint->getVariablePosition(idx[j]);
                q3 = q1;

                dt1 = dt2 = time_diff[i - 1];
            }

            double v1, v2, a;

            bool start_velocity = false;
            if (dt1 == 0.0 || dt2 == 0.0)
            {
                v1 = 0.0;
                v2 = 0.0;
                a = 0.0;
            }
            else
            {
                if (i == 0)
                {
                    if (curr_waypoint->hasVelocities())
                    {
                        start_velocity = true;
                        v1 = curr_waypoint->getVariableVelocity(idx[j]);
                    }
                }
                v1 = start_velocity ? v1 : (q2 - q1) / dt1;
                // v2 = (q3-q2)/dt2;
                v2 = start_velocity ? v1 : (q3 - q2) / dt2; // Needed to ensure continuous velocity for first point
                a = 2.0 * (v2 - v1) / (dt1 + dt2);
            }

            curr_waypoint->setVariableVelocity(idx[j], (v2 + v1) / 2.0);
            curr_waypoint->setVariableAcceleration(idx[j], a);
        }
    }
}
} // namespace

void ConstEESpeedTimeParameterization::applyConstEESpeed(robot_trajectory::RobotTrajectory &rob_trajectory,
                                                         std::vector<double> &time_diff,
                                                         const std::string end_effector_frame,
                                                         const double ee_speed_request) const
{
    const robot_model::JointModelGroup *group = rob_trajectory.getGroup();
    const std::vector<std::string> &vars = group->getVariableNames();
    const std::vector<int> &idx = group->getVariableIndexList();
    const robot_model::RobotModel &rmodel = group->getParentModel();
    const int num_points = rob_trajectory.getWayPointCount();
    int n = 0;

    double ee_speed = 1.0;
    double ee_speed_command = 1.0;

    if (ee_speed_request > 0.0 && ee_speed_request <= 5.0)
        ee_speed = ee_speed_request;
    else if (ee_speed_request == 0.0)
        ROS_DEBUG_NAMED("trajectory_processing.const_ee_speed_time_parameterization",
                        "A ee_speed_request of 0.0 was specified, defaulting to %f instead.",
                        ee_speed);
    else
        ROS_WARN_NAMED("trajectory_processing.const_ee_speed_time_parameterization",
                       "Invalid ee_speed_request %f specified, defaulting to %f instead.",
                       ee_speed_request, ee_speed);

    double ee_dist, t_min;

    for (int i = 0; i < num_points - 1; ++i)
    {
        const robot_state::RobotStatePtr &curr_waypoint = rob_trajectory.getWayPointPtr(i);
        Eigen::Affine3d curr_ee_waypoint = curr_waypoint->getFrameTransform(end_effector_frame);
        const robot_state::RobotStatePtr &next_waypoint = rob_trajectory.getWayPointPtr(i + 1);
        Eigen::Affine3d next_ee_waypoint = next_waypoint->getFrameTransform(end_effector_frame);

        ee_dist = sqrt(pow(next_ee_waypoint.translation()[0] - curr_ee_waypoint.translation()[0], 2) +
                       pow(next_ee_waypoint.translation()[1] - curr_ee_waypoint.translation()[1], 2) +
                       pow(next_ee_waypoint.translation()[2] - curr_ee_waypoint.translation()[2], 2));

        // Enforce a trapezoidal ee velocity profile. Keep line after else if not needed
        if (i < ACCELERATE_NUMBER_POINTS)
        {
            ee_speed_command = ee_speed * (double(i+1)/double(ACCELERATE_NUMBER_POINTS));
        }
        else if (i > num_points - ACCELERATE_NUMBER_POINTS -2)
        {
            ee_speed_command = ee_speed * (1-(double(n)/double(ACCELERATE_NUMBER_POINTS)));
            n = n + 1;
        }
        else
        {
            ee_speed_command = ee_speed;
        }

        // Use trigonometric functions to smooth start and end of the path
        // if (i < ACCELERATE_NUMBER_POINTS)
        // {
        //    ee_speed_command = ee_speed * 0.5*(1+sin(-(M_PI /2.0) + double(i+1)/double(ACCELERATE_NUMBER_POINTS)*M_PI));
        // }
        // else if (i > num_points - ACCELERATE_NUMBER_POINTS -2)
        // {
        //    ee_speed_command = ee_speed * 0.5*(1+cos(double(n)/double(ACCELERATE_NUMBER_POINTS)*M_PI));
        //    n = n + 1;
        // }
        // else
        // {
        //    ee_speed_command = ee_speed;
        // } 

        t_min = ee_dist / ee_speed_command;
        time_diff[i] = t_min;
    }
}

bool ConstEESpeedTimeParameterization::checkEESpeed(robot_trajectory::RobotTrajectory &rob_trajectory,
                                                         std::vector<double> &time_diff,
                                                         const std::string end_effector_frame,
                                                         const double ee_speed_request) const
{
    const robot_model::JointModelGroup *group = rob_trajectory.getGroup();
    const std::vector<std::string> &vars = group->getVariableNames();
    const std::vector<int> &idx = group->getVariableIndexList();
    const robot_model::RobotModel &rmodel = group->getParentModel();
    const int num_points = rob_trajectory.getWayPointCount();

    double ee_dist, delta_time;
    double ee_speed;
    double percentage = 1;
    int num_correct_speed = 0;

    for (int i = ACCELERATE_NUMBER_POINTS-1 ; i < num_points - ACCELERATE_NUMBER_POINTS - 1; ++i)
    {
        const robot_state::RobotStatePtr &curr_waypoint = rob_trajectory.getWayPointPtr(i);
        Eigen::Affine3d curr_ee_waypoint = curr_waypoint->getFrameTransform(end_effector_frame);
        const robot_state::RobotStatePtr &next_waypoint = rob_trajectory.getWayPointPtr(i + 1);
        Eigen::Affine3d next_ee_waypoint = next_waypoint->getFrameTransform(end_effector_frame);

        ee_dist = sqrt(pow(next_ee_waypoint.translation()[0] - curr_ee_waypoint.translation()[0], 2) +
                       pow(next_ee_waypoint.translation()[1] - curr_ee_waypoint.translation()[1], 2) +
                       pow(next_ee_waypoint.translation()[2] - curr_ee_waypoint.translation()[2], 2));

        delta_time = time_diff[i];

        ee_speed = ee_dist / delta_time;

        if (ee_speed >= (1-percentage/100)*ee_speed_request and ee_speed <= (1+percentage/100)*ee_speed_request)
        {
            num_correct_speed  += 1;
        }
    }

    if (num_correct_speed < 0.8*(num_points-2*ACCELERATE_NUMBER_POINTS))
    {
        ROS_WARN_STREAM("Cartesian speed of " << ee_speed_request << " m/s cannot be reach at most of the waypoints, reducing to a smaller value");
        return false;
    }
    else
    {
        ROS_INFO_STREAM("Trajectory will be performed with cartesian speed of " << ee_speed_request << " m/s");
        return true;
    }
    
}


bool ConstEESpeedTimeParameterization::computeTimeStamps(robot_trajectory::RobotTrajectory &trajectory, const std::string end_effector_frame, const double ee_speed_request,
                                                         const double max_velocity_scaling_factor,
                                                         const double max_acceleration_scaling_factor) const
{
    if (trajectory.empty())
        return true;

    const robot_model::JointModelGroup *group = trajectory.getGroup();
    if (!group)
    {
        ROS_ERROR_NAMED("trajectory_processing.iterative_time_parameterization", "It looks like the planner did not set "
                                                                                 "the group the plan was computed for");
        return false;
    }

    // this lib does not actually work properly when angles wrap around, so we need to unwind the path first
    trajectory.unwind();

    const int num_points = trajectory.getWayPointCount();
    std::vector<double> time_diff(num_points - 1, 0.0); // the time difference between adjacent points

    bool ok = false;
    double ee_speed = ee_speed_request;
    int iteration = 0;

    while (!ok)
    {
        applyConstEESpeed(trajectory, time_diff, end_effector_frame, ee_speed);
        applyVelocityConstraints(trajectory, time_diff, max_velocity_scaling_factor);
        applyAccelerationConstraints(trajectory, time_diff, max_acceleration_scaling_factor);
        ok = checkEESpeed(trajectory, time_diff, end_effector_frame, ee_speed);
        ee_speed *= 0.9;
        iteration++;
        if (iteration == 16)
        {
            ROS_ERROR_STREAM("Constant End-effector speed cannot be obtained withput using very small values");
            exit(-1);
        }   
    }


    updateTrajectory(trajectory, time_diff);
    return true;
}
} // namespace layla_motion_planning
