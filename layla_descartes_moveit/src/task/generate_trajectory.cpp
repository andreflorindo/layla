/* Author: Andre Florindo*/

/* Goal: Allocates the trajectory points into the respective descartes trajectory object*/

#include <layla_descartes_moveit_planner.h>

namespace layla_motion_planning
{

void LayLaDescartesMoveitPlanner::generateTrajectory(EigenSTL::vector_Isometry3d &poses, std::vector<descartes_core::TrajectoryPtPtr> &input_traj)
{
    // creating descartes trajectory points
    input_traj.clear();
    input_traj.reserve(poses.size());
    for (unsigned int i = 0; i < poses.size(); i++)
    {
        const Eigen::Isometry3d &single_pose = poses[i];

        //Trajectory with rotational freedom about the tool's z axis.
        // descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
        //     new descartes_trajectory::AxialSymmetricPt(single_pose, ORIENTATION_INCREMENT, descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS));

        //Trajectory utilizing the whole six degree-of-freedom
        descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
            new descartes_trajectory::CartTrajectoryPt(descartes_trajectory::TolerancedFrame(single_pose)));

        input_traj.emplace_back(pt);
    }

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

} // namespace layla_motion_planning
