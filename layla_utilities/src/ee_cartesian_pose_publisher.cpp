/* Author: Andre Florindo*/

#include <ee_cartesian_pose_publisher.h>

namespace layla_motion_planning
{

CartesianPosePublisher::CartesianPosePublisher() {}
CartesianPosePublisher::~CartesianPosePublisher() {}

void CartesianPosePublisher::initTopic()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("tip_link", config_.tip_link) &&
        ph.getParam("base_link", config_.base_link))
    {
        ROS_INFO_STREAM("ee_cartesian_pose_publisher: Loaded Topic parameters");
    }
    else
    {
        ROS_ERROR_STREAM("ee_cartesian_pose_publisher: Failed to load Topic parameters");
        exit(-1);
    }

    cartesian_pose_publisher_ = nh.advertise<geometry_msgs::TransformStamped>(EE_POSE_TOPIC, 1, true);

    ROS_INFO_STREAM("ee_cartesian_pose_publisher: Task '" << __FUNCTION__ << "' completed");
}

void CartesianPosePublisher::startListener()
{
    tf::TransformListener listener;
    ros::Rate rate(50.0);
    tf::StampedTransform transform;
    geometry_msgs::TransformStamped ee_cartesian_pose_msg;

    ros::Duration(1.0).sleep(); //wait until robot is initialized  

    while (nh_.ok())
    {
        try
        {
            listener.lookupTransform(config_.base_link, config_.tip_link,
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        tf::transformStampedTFToMsg(transform, ee_cartesian_pose_msg);
        cartesian_pose_publisher_.publish(ee_cartesian_pose_msg);
    }

    ROS_INFO_STREAM("ee_cartesian_pose_publisher: Task '" << __FUNCTION__ << "' completed");
}

} // namespace layla_motion_planning

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ee_cartesian_pose_publisher");

    layla_motion_planning::CartesianPosePublisher ee_cartesian_pose_publisher;

    ee_cartesian_pose_publisher.initTopic();

    ee_cartesian_pose_publisher.startListener();

    ros::spin();
}