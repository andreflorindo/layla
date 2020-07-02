/* Author: Andre Florindo*/

/* Goal: Send request to the server and receives the course path.
        Also converts the PoseArray message to a tranformation matrix
*/

#include <layla_descartes_moveit_planner.h>

namespace layla_motion_planning
{
bool LayLaDescartesMoveitPlanner::getCourse(EigenSTL::vector_Isometry3d &poses)
{
    // Initialize Service client
    if (ros::service::waitForService(POSE_BUILDER_SERVICE, ros::Duration(SERVER_TIMEOUT)))
    {
        ROS_INFO_STREAM("Connected to '" << POSE_BUILDER_SERVICE << "' service");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to connect to '" << POSE_BUILDER_SERVICE << "' service");
        exit(-1);
    }

    pose_builder_client_ = nh_.serviceClient<layla_msgs::PoseBuilder>(POSE_BUILDER_SERVICE);
    layla_msgs::PoseBuilder srv;
    srv.request.num_layer = config_.layer;
    srv.request.num_course = config_.course;

    if (!pose_builder_client_.call(srv))
    {
        ROS_ERROR_STREAM("Failed to call '" << POSE_BUILDER_SERVICE << "' service");
        exit(-1);
    }

    // Modify the single_pose type from PoseArray to Isometry3d
    Eigen::Isometry3d single_pose;
    poses.reserve(srv.response.single_course_poses.poses.size());

    for (unsigned int i = 0; i < srv.response.single_course_poses.poses.size(); i++)
    {
        tf::poseMsgToEigen(srv.response.single_course_poses.poses[i], single_pose);
        poses.emplace_back(single_pose);
    }

    // Publish the response course in order to send it to Rviz
    course_publisher_.publish(srv.response.single_course_poses); 
    
    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");


}
} // namespace layla_motion_planning

