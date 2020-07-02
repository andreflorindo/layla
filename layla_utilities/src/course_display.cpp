/* Author: Andre Florindo*/

// Add vector<CourseStruct> in case there are more courses

#include <course_display_topic.h>

namespace layla_motion_planning
{

CourseDisplay::CourseDisplay() {}
CourseDisplay::~CourseDisplay() {}

void CourseDisplay::initTopic()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    course_subscriber_ = nh.subscribe("single_course_poses", 1, &CourseDisplay::subscriberCallback, this); 
    marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC, 1, true);

    ROS_INFO_STREAM("course_display: Task '" << __FUNCTION__ << "' completed");
}

void CourseDisplay::subscriberCallback(const geometry_msgs::PoseArray &msg)
{
    if(msg.header.frame_id.empty())
    {
        ROS_ERROR("course_display: Topic single_course_poses does not specify Frame ID");
        exit(-1);
    }
    else
    {
        ROS_INFO_STREAM("course_display: Joint trajectory received");
    }
    publishPosesMarkers(msg);
}

void CourseDisplay::publishPosesMarkers(const geometry_msgs::PoseArray &course_poses)
{
    // creating rviz markers
    visualization_msgs::Marker z_axes, y_axes, x_axes, line;
    z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::LINE_LIST;
    //z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::ARROW;
    z_axes.ns = y_axes.ns = x_axes.ns = "axes";
    z_axes.action = y_axes.action = x_axes.action = visualization_msgs::Marker::ADD;
    z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);
    z_axes.header.frame_id = y_axes.header.frame_id = x_axes.header.frame_id = course_poses.header.frame_id;
    z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = ARROW_WIDTH;

    // z properties
    z_axes.id = 0;
    z_axes.color.r = 0;
    z_axes.color.g = 0;
    z_axes.color.b = 1;
    z_axes.color.a = 1;

    // y properties
    y_axes.id = 1;
    y_axes.color.r = 0;
    y_axes.color.g = 1;
    y_axes.color.b = 0;
    y_axes.color.a = 1;

    // x properties
    x_axes.id = 2;
    x_axes.color.r = 1;
    x_axes.color.g = 0;
    x_axes.color.b = 0;
    x_axes.color.a = 1;

    // line properties
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.ns = "line";
    line.action = visualization_msgs::Marker::ADD;
    line.lifetime = ros::Duration(0);
    line.header.frame_id = course_poses.header.frame_id;
    line.scale.x = AXIS_LINE_WIDTH;
    line.id = 0;
    line.color.r = 1;
    line.color.g = 1;
    line.color.b = 0;
    line.color.a = 1;

    // creating axes markers

    int npoints = course_poses.poses.size();

    z_axes.points.reserve(2 * npoints);
    y_axes.points.reserve(2 * npoints);
    x_axes.points.reserve(2 * npoints);
    line.points.reserve(npoints);
    geometry_msgs::Point p_start, p_end;
    double distance = 0;

    EigenSTL::vector_Isometry3d poses;
    Eigen::Isometry3d single_pose;
    poses.reserve(npoints);

    for (unsigned int i = 0; i < npoints; i++)
    {
        tf::poseMsgToEigen(course_poses.poses[i], single_pose);
        poses.emplace_back(single_pose);
    }

    Eigen::Isometry3d prev = poses[0];
    for (unsigned int i = 0; i < npoints; i++)
    {
        const Eigen::Isometry3d &pose = poses[i];
        distance = (pose.translation() - prev.translation()).norm();

        tf::pointEigenToMsg(pose.translation(), p_start);

        if (distance > ARROW_LENGHT || i == 0)
        {
            Eigen::Isometry3d moved_along_x = pose * Eigen::Translation3d(ARROW_LENGHT, 0, 0);
            tf::pointEigenToMsg(moved_along_x.translation(), p_end);
            x_axes.points.emplace_back(p_start);
            x_axes.points.emplace_back(p_end);

            Eigen::Isometry3d moved_along_y = pose * Eigen::Translation3d(0, ARROW_LENGHT, 0);
            tf::pointEigenToMsg(moved_along_y.translation(), p_end);
            y_axes.points.emplace_back(p_start);
            y_axes.points.emplace_back(p_end);

            Eigen::Isometry3d moved_along_z = pose * Eigen::Translation3d(0, 0, ARROW_LENGHT);
            tf::pointEigenToMsg(moved_along_z.translation(), p_end);
            z_axes.points.emplace_back(p_start);
            z_axes.points.emplace_back(p_end);

            // saving previous
            prev = pose;
        }

        line.points.emplace_back(p_start);
    }

    markers_msg_.markers.push_back(x_axes);
    markers_msg_.markers.push_back(y_axes);
    markers_msg_.markers.push_back(z_axes);
    markers_msg_.markers.push_back(line);

    marker_publisher_.publish(markers_msg_);
}

} // namespace layla_motion_planning

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_builder");

    layla_motion_planning::CourseDisplay course_display;

    course_display.initTopic();
    
    ros::spin();
}