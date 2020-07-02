/* Author: Andre Florindo*/

#include <ee_request_publisher.h>

namespace layla_motion_planning
{

EERequestPublisher::EERequestPublisher() {}
EERequestPublisher::~EERequestPublisher() {}

void EERequestPublisher::initTopic()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("group_name", config_.group_name) &&
        ph.getParam("tip_link", config_.tip_link) &&
        ph.getParam("base_link", config_.base_link))
    {
        ROS_INFO_STREAM("ee_request_publisher: Loaded Topic parameters");
    }
    else
    {
        ROS_ERROR_STREAM("ee_request_publisher: Failed to load Topic parameters");
        exit(-1);
    }

    loadRobotModel();

    joint_request_subscriber_ = nh.subscribe("joint_request", 1000, &EERequestPublisher::subscriberCallback, this);

    ee_request_publisher_ = nh.advertise<layla_msgs::EERequest>("ee_request", 1000, true);

    ROS_INFO_STREAM("ee_request_publisher: Task '" << __FUNCTION__ << "' completed");
}

void EERequestPublisher::subscriberCallback(const layla_msgs::JointRequest &msg)
{
    joint_request_ = msg;
    publishEERequest();
}

void EERequestPublisher::loadRobotModel()
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

void EERequestPublisher::publishEERequest()
{
    layla_msgs::EERequest ee_request;
    kinematic_state_->setJointGroupPositions(config_.group_name, joint_request_.position);

    ee_request.header = joint_request_.header;
    ee_request.position = getEEPosition();
    ee_request.velocity = getEEVelocity();
    ee_request.combined_linear_velocity = getEELinearVelocity(ee_request.velocity);
    // ee_request.acceleration = getEEAcceleration();
    // ee_request.jerk = getEEJerk();
    ee_request_publisher_.publish(ee_request);

}

geometry_msgs::Transform EERequestPublisher::getEEPosition()
{
    geometry_msgs::Transform ee_cartesian_pose_msg;

    Eigen::Affine3d root_to_world = kinematic_state_->getFrameTransform(config_.base_link);
    Eigen::Affine3d world_to_root = root_to_world.inverse();

    Eigen::Affine3d transform = world_to_root * kinematic_state_->getFrameTransform(config_.tip_link);
    tf::transformEigenToMsg(transform, ee_cartesian_pose_msg);

    return ee_cartesian_pose_msg;
}

geometry_msgs::Twist EERequestPublisher::getEEVelocity()
{
    Eigen::MatrixXd jacobian;
    geometry_msgs::Twist msg_ee_velocities;

    double *ptr = &joint_request_.velocity[0];
    Eigen::Map<Eigen::VectorXd> joints_speed(ptr, joint_request_.velocity.size());

    getJacobian(jacobian);
    Eigen::VectorXd ee_velocities = jacobian * joints_speed;
    
    msg_ee_velocities.linear.x=ee_velocities[0];
    msg_ee_velocities.linear.y=ee_velocities[1];
    msg_ee_velocities.linear.z=ee_velocities[2];
    msg_ee_velocities.angular.x=ee_velocities[3];
    msg_ee_velocities.angular.y=ee_velocities[4];
    msg_ee_velocities.angular.z=ee_velocities[5];
    return msg_ee_velocities;
}

void EERequestPublisher::getJacobian(Eigen::MatrixXd &jacobian)
{
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);

    kinematic_state_->getJacobian(joint_model_group_, kinematic_state_->getLinkModel(joint_model_group_->getLinkModelNames().back()),
                                  reference_point_position,
                                  jacobian);
}

double EERequestPublisher::getEELinearVelocity(geometry_msgs::Twist &ee_request_velocity)
{
    double ee_linear_velocity;

    ee_linear_velocity = sqrt(pow(ee_request_velocity.linear.x, 2.0) +
                              pow(ee_request_velocity.linear.y, 2.0) +
                              pow(ee_request_velocity.linear.z, 2.0));

    return ee_linear_velocity;
}

// std::vector<double> EERequestPublisher::getEEAcceleration()
// {
//     Eigen::MatrixXd jacobian;

//     getJacobian(jacobian);

//     std::vector<double> ee_velocities = jacobian * joint_request_.velocity;

//     return ee_velocities;
// }

} // namespace layla_motion_planning

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ee_request_publisher");

    layla_motion_planning::EERequestPublisher ee_request_publisher;

    ee_request_publisher.initTopic();

    ros::spin();
}