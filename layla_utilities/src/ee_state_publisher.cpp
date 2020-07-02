/* Author: Andre Florindo*/

#include <ee_state_publisher.h>

namespace layla_motion_planning
{

EEStatePublisher::EEStatePublisher() {}
EEStatePublisher::~EEStatePublisher() {}

void EEStatePublisher::initTopic()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("group_name", config_.group_name) &&
        ph.getParam("tip_link", config_.tip_link) &&
        ph.getParam("base_link", config_.base_link))
    {
        ROS_INFO_STREAM("ee_state_publisher: Loaded Topic parameters");
    }
    else
    {
        ROS_ERROR_STREAM("ee_state_publisher: Failed to load Topic parameters");
        exit(-1);
    }

    loadRobotModel();
    count_ = 0;

    // ros::Duration(15.0).sleep(); //wait until robot is initialized 

    joint_states_subscriber_ = nh.subscribe("joint_states", 1000, &EEStatePublisher::subscriberCallback, this);

    ee_state_publisher_ = nh.advertise<layla_msgs::EERequest>("ee_states", 1000, true);

    ROS_INFO_STREAM("ee_state_publisher: Task '" << __FUNCTION__ << "' completed");
}

void EEStatePublisher::subscriberCallback(const sensor_msgs::JointState &msg)
{
    joint_states_ = msg;
    if (count_ == 0)
    {
        stateMsgToEigen(joint_states_, prev_joints_position_);
        prev_time_ = joint_states_.header.stamp.toSec();
    }   
    publishEEState();
    count_++;
}

void EEStatePublisher::stateMsgToEigen(const sensor_msgs::JointState &m, Eigen::VectorXd &e)
{
    e.resize(m.position.size());

    for (size_t i = 0; i < m.position.size() ; i++)
    {
        e(i) = m.position[i];
    }    
}

void EEStatePublisher::loadRobotModel()
{
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    kinematic_model_ = robot_model_loader_->getModel();
    if (!kinematic_model_)
    {
        ROS_ERROR_STREAM("Failed to load robot model from robot description parameter:robot_description");
        exit(-1);
    }

    joint_model_group_ = kinematic_model_->getJointModelGroup(config_.group_name);
    kinematic_state_.reset(new moveit::core::RobotState(kinematic_model_));
}

void EEStatePublisher::publishEEState()
{
    layla_msgs::EERequest ee_state;
    kinematic_state_->setJointGroupPositions(config_.group_name, joint_states_.position);

    ee_state.header = joint_states_.header;
    ee_state.position = getEEPosition();
    ee_state.velocity = getEEVelocity();
    ee_state.combined_linear_velocity = getEELinearVelocity(ee_state.velocity);
    // ee_state.acceleration = getEEAcceleration();
    // ee_state.jerk = getEEJerk();
    ee_state_publisher_.publish(ee_state);

}

geometry_msgs::Transform EEStatePublisher::getEEPosition()
{
    geometry_msgs::Transform ee_cartesian_pose_msg;

    Eigen::Affine3d root_to_world = kinematic_state_->getFrameTransform(config_.base_link);
    Eigen::Affine3d world_to_root = root_to_world.inverse();

    Eigen::Affine3d transform = world_to_root * kinematic_state_->getFrameTransform(config_.tip_link);
    tf::transformEigenToMsg(transform, ee_cartesian_pose_msg);

    return ee_cartesian_pose_msg;
}

geometry_msgs::Twist EEStatePublisher::getEEVelocity()
{
    Eigen::MatrixXd jacobian;
    Eigen::VectorXd joints_speed(joint_states_.position.size());
    geometry_msgs::Twist msg_ee_velocities;

    getJacobian(jacobian);
    getJointsSpeed(joints_speed);

    Eigen::VectorXd ee_velocities = jacobian * joints_speed;
    
    msg_ee_velocities.linear.x=ee_velocities[0];
    msg_ee_velocities.linear.y=ee_velocities[1];
    msg_ee_velocities.linear.z=ee_velocities[2];
    msg_ee_velocities.angular.x=ee_velocities[3];
    msg_ee_velocities.angular.y=ee_velocities[4];
    msg_ee_velocities.angular.z=ee_velocities[5];
    return msg_ee_velocities;
}

void EEStatePublisher::getJacobian(Eigen::MatrixXd &jacobian)
{
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);

    kinematic_state_->getJacobian(joint_model_group_, kinematic_state_->getLinkModel(joint_model_group_->getLinkModelNames().back()),
                                  reference_point_position,
                                  jacobian);
}

void EEStatePublisher::getJointsSpeed(Eigen::VectorXd &joints_speed)
{
    Eigen::VectorXd delta_theta(joint_states_.position.size());
    Eigen::VectorXd cur_joint_position(joint_states_.position.size());
    double delta_time;
    if (count_ == 0)
    {
        joints_speed = Eigen::VectorXd::Zero(joint_states_.position.size());
    }
    else
    {
        stateMsgToEigen(joint_states_, cur_joint_position);
        delta_theta =  cur_joint_position - prev_joints_position_;
        delta_time = joint_states_.header.stamp.toSec() - prev_time_;
        joints_speed = delta_theta / delta_time;
    
    }
    prev_joints_position_ = cur_joint_position;
    prev_time_ = joint_states_.header.stamp.toSec();
}



double EEStatePublisher::getEELinearVelocity(geometry_msgs::Twist &ee_state_velocity)
{
    double ee_linear_velocity;

    ee_linear_velocity = sqrt(pow(ee_state_velocity.linear.x, 2.0) +
                              pow(ee_state_velocity.linear.y, 2.0) +
                              pow(ee_state_velocity.linear.z, 2.0));

    return ee_linear_velocity;
}

} // namespace layla_motion_planning


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ee_state_publisher");

    layla_motion_planning::EEStatePublisher ee_state_publisher;

    ee_state_publisher.initTopic();

    ros::spin();
}