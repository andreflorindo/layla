/* Author: Andre Florindo*/
#ifndef EE_CARTESIAN_POSE_PUBLISHER_H
#define EE_CARTESIAN_POSE_PUBLISHER_H

// ROS
#include <ros/ros.h>

//TF
#include <tf/transform_listener.h>

namespace layla_motion_planning
{

const std::string EE_POSE_TOPIC = "ee_cartesian_pose";

struct CartesianPosePublisherConfiguration
{
    std::string tip_link;
    std::string base_link;
};

class CartesianPosePublisher
{
public:
    CartesianPosePublisher();
    virtual ~CartesianPosePublisher();

    void initTopic();
    void startListener();

protected:
    CartesianPosePublisherConfiguration config_;
    ros::NodeHandle nh_;
    ros::Publisher cartesian_pose_publisher_;
};

} // namespace layla_motion_planning

#endif