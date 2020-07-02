/* Author: Andre Florindo*/

/* Goal: Load parameters from the ros parameter server.
        Start Moveit execute_trajectory action client and a publisher of the cartesian poses
*/

#include <layla_descartes_moveit_planner.h>

namespace layla_motion_planning
{

LayLaDescartesMoveitPlanner::LayLaDescartesMoveitPlanner() {}
LayLaDescartesMoveitPlanner::~LayLaDescartesMoveitPlanner() {}

void LayLaDescartesMoveitPlanner::initRos()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("group_name", config_.group_name) &&
        ph.getParam("tip_link", config_.tip_link) &&
        ph.getParam("base_link", config_.base_link) &&
        ph.getParam("world_frame", config_.world_frame) &&
        ph.getParam("trajectory/seed_pose", config_.seed_pose) &&
        ph.getParam("max_joint_speed_scaling_between_traj", config_.max_joint_speed_scaling_between_traj) &&
        ph.getParam("ee_speed", config_.ee_speed) &&
        ph.getParam("layer", config_.layer) &&
        ph.getParam("course", config_.course) &&
        nh.getParam("controller_joint_names", config_.joint_names))
    {
        ROS_INFO_STREAM("Loaded application parameters");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load application parameters");
        exit(-1);
    }

    typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> client_type;
    moveit_run_path_client_ptr_ = std::make_shared<client_type>(EXECUTE_TRAJECTORY_ACTION, true);

    // Establishing connection to server
    if (moveit_run_path_client_ptr_->waitForServer(ros::Duration(SERVER_TIMEOUT)))
    {
        ROS_INFO_STREAM("Connected to '" << EXECUTE_TRAJECTORY_ACTION << "' action");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to connect to '" << EXECUTE_TRAJECTORY_ACTION << "' action");
        exit(-1);
    }

    course_publisher_ = nh.advertise<geometry_msgs::PoseArray>("single_course_poses", 1, true);

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

} // namespace layla_motion_planning
