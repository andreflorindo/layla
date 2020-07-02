/* Author: Andre Florindo*/
#ifndef EE_STATE_PUBLISHER_H
#define EE_STATE_PUBLISHER_H

// ROS
#include <ros/ros.h>

//msg
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

struct EEStatePublisherConfiguration
{
    std::string group_name;
    std::string tip_link;
    std::string base_link;
};

class EEStatePublisher
{
public:
    EEStatePublisher();
    virtual ~EEStatePublisher();

    void initTopic();
    void subscriberCallback(const sensor_msgs::JointState &msg);
    void publishEEState();
    void loadRobotModel();
    geometry_msgs::Transform getEEPosition();
    geometry_msgs::Twist getEEVelocity();
    double getEELinearVelocity(geometry_msgs::Twist &ee_state_velocity);
    void getJacobian(Eigen::MatrixXd &jacobian);
    void getJointsSpeed(Eigen::VectorXd &joints_speed);

    sensor_msgs::JointState joint_states_;

protected:
    EEStatePublisherConfiguration config_;
    ros::NodeHandle nh_;
    ros::Publisher ee_state_publisher_;
    ros::Subscriber joint_states_subscriber_;
    void stateMsgToEigen(const sensor_msgs::JointState &m, Eigen::VectorXd &e);

    //robot model
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    mutable moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup *joint_model_group_;
    robot_model::RobotModelConstPtr kinematic_model_;
    int count_;
    Eigen::VectorXd prev_joints_position_;
    double prev_time_;
};

} // namespace layla_motion_planning

#endif