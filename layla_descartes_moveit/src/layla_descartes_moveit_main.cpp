/* Author: Andre Florindo*/

#include <layla_descartes_moveit_planner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "layla_descartes_moveit_main");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Main program
    layla_motion_planning::LayLaDescartesMoveitPlanner planner;

    planner.initRos();

    EigenSTL::vector_Isometry3d poses;
    planner.getCourse(poses);

    planner.initDescartes();

    std::vector<descartes_core::TrajectoryPtPtr> input_traj;
    planner.generateTrajectory(poses, input_traj);

    std::vector<descartes_core::TrajectoryPtPtr> output_traj;
    moveit_msgs::RobotTrajectory moveit_traj;
    planner.planPath(input_traj, output_traj, moveit_traj);

    //ros::Duration(5).sleep();

    planner.runPath(output_traj, moveit_traj);

    spinner.stop();
    return 0;
}