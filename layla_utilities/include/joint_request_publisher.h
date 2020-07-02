/* Author: Andre Florindo*/
#ifndef JOINT_REQUEST_PUBLISHER_H
#define JOINT_REQUEST_PUBLISHER_H

// ROS
#include <ros/ros.h>

//msg
#include <layla_msgs/JointRequest.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>

namespace layla_motion_planning
{

//const std::string EE_VELOCITY_TOPIC = "ee_velocity";

struct JointRequestPublisherConfiguration
{
    std::string tip_link;
    std::string base_link;
};

class JointRequestPublisher
{
public:
    JointRequestPublisher();
    virtual ~JointRequestPublisher();

    void initTopic();
    void subscriberCallbackSim(const trajectory_msgs::JointTrajectory &msg);
    void subscriberCallbackReal(const control_msgs::FollowJointTrajectoryFeedback &msg);
    void checkJointPathCommand();
    void publishJointRequest();

    trajectory_msgs::JointTrajectory joint_path_;
    int total_num_points = 0;
    int seq = 0;
    ros::Time time_point;

protected:
    JointRequestPublisherConfiguration config_;
    ros::NodeHandle nh_;
    ros::Publisher joint_request_publisher_;
    ros::Subscriber joint_path_subscriber_;
};

} // namespace layla_motion_planning

#endif