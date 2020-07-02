/* Author: Andre Florindo*/

#include <joint_request_publisher.h>

namespace layla_motion_planning
{

JointRequestPublisher::JointRequestPublisher() {}
JointRequestPublisher::~JointRequestPublisher() {}

void JointRequestPublisher::initTopic()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    bool sim;

    if (ph.getParam("sim", sim))
    {
        ROS_INFO_STREAM("ee_request_publisher: Loaded Topic parameters");
    }

    if (sim)
        joint_path_subscriber_ = nh.subscribe("joint_path_command", 1000, &JointRequestPublisher::subscriberCallbackSim, this);
    else
        joint_path_subscriber_ = nh.subscribe("feedback_states", 1000, &JointRequestPublisher::subscriberCallbackReal, this);


    joint_request_publisher_ = nh.advertise<layla_msgs::JointRequest>("joint_request", 1000, true);

    ROS_INFO_STREAM("joint_request_publisher: Task '" << __FUNCTION__ << "' completed");
}

void JointRequestPublisher::subscriberCallbackSim(const trajectory_msgs::JointTrajectory &msg)
{
    seq++;
    time_point = ros::Time::now();
    joint_path_ = msg;
    ROS_INFO("joint_request_publisher: Joint trajectory %d received", seq);
    checkJointPathCommand();
    publishJointRequest();
}

void JointRequestPublisher::subscriberCallbackReal(const control_msgs::FollowJointTrajectoryFeedback &msg)
{
    layla_msgs::JointRequest joint_request;
    joint_request.header = msg.header;
    joint_request.position = msg.desired.positions;
    joint_request.velocity = msg.desired.velocities;
    joint_request.acceleration = msg.desired.accelerations;
    joint_request_publisher_.publish(joint_request);
}

void JointRequestPublisher::checkJointPathCommand()
{

    if (joint_path_.points[0].positions.empty())
    {
        ROS_ERROR_STREAM("joint_request_publisher: Joint path is not available in the topic joint_path_command");
        exit(-1);
    }

    int num_joints = joint_path_.points[0].positions.size();
    int num_points = joint_path_.points.size();

    if (joint_path_.points[0].velocities.empty())
    {
        ROS_WARN_STREAM("joint_request_publisher: Joint velocity is not given in the topic joint_path_command, it will be assumed as 0");
        for (int j = 0; j < num_points; j++)
        {
            for (int i = 0; i < num_joints; i++)
            {
                joint_path_.points[j].velocities.push_back(0.0f);
            }
        }
    }

    if (joint_path_.points[0].accelerations.empty())
    {
        ROS_WARN_STREAM("joint_request_publisher: Joint acceleration is not given in the topic joint_path_command, it will be assumed as 0");
        for (int j = 0; j < num_points; j++)
        {
            for (int i = 0; i < num_joints; i++)
            {
                joint_path_.points[j].accelerations.push_back(0.0f);
            }
        }
    }
}

void JointRequestPublisher::publishJointRequest()
{
    std::vector<std::vector<double>> buffer_position;
    std::vector<std::vector<double>> buffer_velocity;
    std::vector<std::vector<double>> buffer_acceleration;
    std::vector<std::vector<double>> buffer_jerk;
    std::vector<double> buffer_time;

    layla_msgs::JointRequest joint_request;

    int num_joints = joint_path_.points[0].positions.size();
    int num_points = joint_path_.points.size();

    buffer_position.resize(num_points, std::vector<double>(num_joints));
    buffer_velocity.resize(num_points, std::vector<double>(num_joints));
    buffer_acceleration.resize(num_points, std::vector<double>(num_joints));
    buffer_jerk.resize(num_points, std::vector<double>(num_joints));
    buffer_time.resize(num_points - 1);

    for (int j = 0; j < num_points - 1; j++)
    {
        buffer_time[j] = joint_path_.points[j + 1].time_from_start.toSec() - joint_path_.points[j].time_from_start.toSec();
    }

    for (int j = 0; j < num_points; j++)
    {
        for (int i = 0; i < num_joints; i++)
        {
            buffer_position[j][i] = joint_path_.points[j].positions[i];
            buffer_velocity[j][i] = joint_path_.points[j].velocities[i];
            buffer_acceleration[j][i] = joint_path_.points[j].accelerations[i];
            buffer_jerk[j][i] = 0.0f;
        }
    }

    for (int j = 0; j < num_points; j++)
    {
        if (j != 0)
            time_point = joint_request.header.stamp + ros::Duration(buffer_time[j - 1]);

        joint_request.header.seq = total_num_points;
        joint_request.header.stamp = time_point;
        joint_request.name = joint_path_.joint_names;
        joint_request.position = buffer_position[j];
        joint_request.velocity = buffer_velocity[j];
        joint_request.acceleration = buffer_acceleration[j];
        joint_request.jerk = buffer_jerk[j];

        joint_request_publisher_.publish(joint_request);
        total_num_points++;
    }

    ROS_INFO_STREAM("joint_request_publisher: Task '" << __FUNCTION__ << "' completed");
}

} // namespace layla_motion_planning

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_request_publisher");

    layla_motion_planning::JointRequestPublisher joint_request_publisher;

    joint_request_publisher.initTopic();

    ros::spin();
}