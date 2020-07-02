#include <layla_ompl_moveit_planner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "layla_ompl_moveit_main");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    layla_motion_planning::LayLaOMPLMoveitPlanner planner;

    planner.initOmpl();

    std::vector<geometry_msgs::Pose> waypoints;
    planner.getCourse(waypoints);

    planner.createMotionPlanRequest(waypoints);

    return 0;
}