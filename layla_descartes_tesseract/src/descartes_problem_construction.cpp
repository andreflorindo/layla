#include <layla_descartes_tesseract_planner.h>

using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_motion_planners;
using namespace tesseract_kinematics;
using namespace opw_kinematics;
using namespace descartes_light;
using namespace descartes_core;
using namespace tesseract_rosutils;

namespace layla_motion_planner
{

LayLaDescartesTesseractPlanner::LayLaDescartesTesseractPlanner() : tesseract_(std::make_shared<tesseract::Tesseract>()) {}
LayLaDescartesTesseractPlanner::~LayLaDescartesTesseractPlanner() {}

void LayLaDescartesTesseractPlanner::initRos()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("group_name", config_.group_name) &&
        ph.getParam("tip_link", config_.tip_link) &&
        ph.getParam("base_link", config_.base_link) &&
        ph.getParam("world_frame", config_.world_frame) &&
        ph.getParam("layer", config_.layer) &&
        ph.getParam("course", config_.course) &&
        ph.getParam("plotting", plotting_) &&
        ph.getParam("rviz", rviz_))
    {
        ROS_INFO_STREAM("Loaded application parameters");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load application parameters");
        exit(-1);
    }

    course_publisher_ = nh.advertise<geometry_msgs::PoseArray>("single_course_poses", 1, true);

    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client_type;
    follow_joint_trajectory_client_ = std::make_shared<client_type>(FOLLOW_JOINT_TRAJECTORY_ACTION, true);

    // Establishing connection to server
    if (follow_joint_trajectory_client_->waitForServer(ros::Duration(SERVER_TIMEOUT)))
    {
        ROS_INFO_STREAM("Connected to '" << FOLLOW_JOINT_TRAJECTORY_ACTION << "' action");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to connect to '" << FOLLOW_JOINT_TRAJECTORY_ACTION << "' action");
        exit(-1);
    }

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

std::vector<Waypoint::Ptr> LayLaDescartesTesseractPlanner::getCourse()
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
    // ROS_INFO_STREAM("Requesting pose in base frame: " << num_layer);

    if (!pose_builder_client_.call(srv))
    {
        ROS_ERROR_STREAM("Failed to call '" << POSE_BUILDER_SERVICE << "' service");
        exit(-1);
    }

    Eigen::Isometry3d single_pose;
    CartesianWaypoint::Ptr waypoint;
    std::vector<Waypoint::Ptr> waypoints;

    // Modify the single_pose type from PoseArray to Isometry3d

    for (unsigned int i = 0; i < srv.response.single_course_poses.poses.size(); i++)
    {
        tf::poseMsgToEigen(srv.response.single_course_poses.poses[i], single_pose);
        waypoint = std::make_shared<CartesianWaypoint>(single_pose, config_.base_link);
        waypoint->setIsCritical(true);
        Eigen::VectorXd c(6);
        c << 1, 1, 1, 1, 1, AXIAL_SYMMETRIC_MOTION;
        waypoint->setCoefficients(c);
        waypoints.push_back(waypoint);
    }

    course_publisher_.publish(srv.response.single_course_poses);

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");

    return waypoints;
}

DescartesMotionPlannerConfigD LayLaDescartesTesseractPlanner::createDescartesPlannerConfig(const Tesseract::ConstPtr &tesseract_ptr,
                                                                                         const std::string & /*manip*/,
                                                                                         const InverseKinematics::ConstPtr &kin,
                                                                                         const double robot_reach,
                                                                                         const EnvState::ConstPtr &current_state,
                                                                                         const std::vector<Waypoint::Ptr> &waypoints,
                                                                                         bool use_collision_edge_evaluator)
{
    const std::vector<std::string> &joint_names = kin->getJointNames();
    const std::vector<std::string> &active_link_names = kin->getActiveLinkNames();
    Eigen::Isometry3d tcp = current_state->transforms.at(kin->getTipLinkName());
    // Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();

    tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
        tesseract_ptr->getEnvironmentConst()->getSceneGraph(), active_link_names, current_state->transforms);

    // Create Collision Interface
    typename descartes_light::CollisionInterface<double>::Ptr coll_interface =
        std::make_shared<DescartesCollision<double>>(
            tesseract_ptr->getEnvironmentConst(), adjacency_map->getActiveLinkNames(), joint_names);

    // Create Timing Constraint
    std::vector<descartes_core::TimingConstraint<double>> timing = makeTiming<double>(waypoints, std::numeric_limits<double>::max());

    // Create Edge Evaluator
    descartes_light::EdgeEvaluator<double>::Ptr edge_computer;
    if (!use_collision_edge_evaluator)
    {
        edge_computer = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>(kin->numJoints());
    }
    else
    {
        edge_computer = std::make_shared<DescartesCollisionEdgeEvaluator<double>>(
            tesseract_ptr->getEnvironmentConst(), adjacency_map->getActiveLinkNames(), joint_names);
    }

    // Create is valid function
    DescartesIsValidFn<double> is_valid_fn =
        std::bind(&tesseract_kinematics::isWithinLimits<double>, std::placeholders::_1, kin->getLimits());

    // Create Position Samplers
    std::vector<typename descartes_light::PositionSampler<double>::Ptr> position_samplers =
        makeRobotSamplers<double>(waypoints, kin, coll_interface, current_state, tcp, robot_reach, true, is_valid_fn);

    return DescartesMotionPlannerConfigD(tesseract_ptr, adjacency_map->getActiveLinkNames(), joint_names, edge_computer, timing, position_samplers, waypoints);
}

trajectory_msgs::JointTrajectory LayLaDescartesTesseractPlanner::trajArrayToJointTrajectoryMsg(std::vector<std::string> joint_names,
                                                                                             tesseract_common::TrajArray traj_array,
                                                                                             bool use_time,
                                                                                             ros::Duration time_increment)
{
    // Create the joint trajectory
    trajectory_msgs::JointTrajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "0";
    traj_msg.joint_names = joint_names;

    tesseract_common::TrajArray pos_mat;
    tesseract_common::TrajArray time_mat;

    if (use_time)
    {
        // Seperate out the time data in the last column from the joint position data
        pos_mat = traj_array.leftCols(traj_array.cols());
        time_mat = traj_array.rightCols(1);
    }
    else
    {
        pos_mat = traj_array;
    }

    ros::Duration time_from_start(0);
    for (int ind = 0; ind < traj_array.rows(); ind++)
    {
        // Create trajectory point
        trajectory_msgs::JointTrajectoryPoint traj_point;

        // Set the position for this time step
        auto mat = pos_mat.row(ind);
        std::vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
        traj_point.positions = vec;

        // Add the current dt to the time_from_start
        if (use_time)
        {
            time_from_start += ros::Duration(time_mat(ind, time_mat.cols() - 1));
        }
        else
        {
            time_from_start += time_increment;
        }
        traj_point.time_from_start = time_from_start;

        traj_msg.points.push_back(traj_point);
    }
    return traj_msg;
}

} // namespace layla_motion_planner