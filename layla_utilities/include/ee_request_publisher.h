/* Author: Andre Florindo*/
#ifndef EE_REQUEST_PUBLISHER_H
#define EE_REQUEST_PUBLISHER_H

// ROS
#include <ros/ros.h>

//msg
#include <layla_msgs/JointRequest.h>
#include <layla_msgs/EERequest.h>

//moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

//Eigen
#include <eigen_conversions/eigen_msg.h>

namespace layla_motion_planning
{

//const std::string EE_VELOCITY_TOPIC = "ee_velocity";

struct EERequestPublisherConfiguration
{
    std::string group_name;
    std::string tip_link;
    std::string base_link;
};

class EERequestPublisher
{
public:
    EERequestPublisher();
    virtual ~EERequestPublisher();

    void initTopic();
    void subscriberCallback(const layla_msgs::JointRequest &msg);
    void publishEERequest();
    void loadRobotModel();
    geometry_msgs::Transform getEEPosition();
    geometry_msgs::Twist getEEVelocity();
    double getEELinearVelocity(geometry_msgs::Twist &ee_request_velocity);
    void getJacobian(Eigen::MatrixXd &jacobian);

    layla_msgs::JointRequest joint_request_;

protected:
    EERequestPublisherConfiguration config_;
    ros::NodeHandle nh_;
    ros::Publisher ee_request_publisher_;
    ros::Subscriber joint_request_subscriber_;

    //robot model
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    mutable moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup *joint_model_group_;
    robot_model::RobotModelConstPtr kinematic_model_;
};

} // namespace layla_motion_planning

#endif