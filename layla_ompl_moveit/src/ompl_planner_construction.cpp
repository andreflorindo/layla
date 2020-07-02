#include <layla_ompl_moveit_planner.h>

namespace layla_motion_planning
{

void LayLaOMPLMoveitPlanner::initOmpl()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("group_name", config_.group_name) &&
        ph.getParam("tip_link", config_.tip_link) &&
        ph.getParam("base_link", config_.base_link) &&
        ph.getParam("world_frame", config_.world_frame) &&
        ph.getParam("trajectory/seed_pose", config_.seed_pose) &&
        nh.getParam("controller_joint_names", config_.joint_names) &&
        ph.getParam("planner_id", config_.planner_id) &&
        ph.getParam("layer", config_.layer) &&
        ph.getParam("course", config_.course) &&
        ph.getParam("max_joint_speed_scaling_between_traj", config_.max_joint_speed_scaling_between_traj) &&
        ph.getParam("ee_speed", config_.ee_speed) &&
        ph.getParam("distance_waypoints", config_.distance_waypoints))
    {
        ROS_INFO_STREAM("Loaded application parameters");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load application parameters");
        exit(-1);
    }

    loadRobotModel();

    course_publisher_ = nh.advertise<geometry_msgs::PoseArray>("single_course_poses", 1, true);

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

void LayLaOMPLMoveitPlanner::loadRobotModel()
{
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));

    kinematic_model_ = robot_model_loader_->getModel();
    if (!kinematic_model_)
    {
        ROS_ERROR_STREAM("Failed to load robot model from robot description parameter:robot_description");
        exit(-1);
    }
    joint_model_group_ = kinematic_model_->getJointModelGroup(config_.group_name);
    kinematic_state_.reset(new moveit::core::RobotState(kinematic_model_));
}

void LayLaOMPLMoveitPlanner::getCourse(std::vector<geometry_msgs::Pose> &poses)
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

    if (!pose_builder_client_.call(srv))
    {
        ROS_ERROR_STREAM("Failed to call '" << POSE_BUILDER_SERVICE << "' service");
        exit(-1);
    }

    for (unsigned int i = 0; i < srv.response.single_course_poses.poses.size(); i++)
    {
        geometry_msgs::Pose single_pose = srv.response.single_course_poses.poses[i];
        poses.emplace_back(single_pose);
    }

    course_publisher_.publish(srv.response.single_course_poses); 

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

void LayLaOMPLMoveitPlanner::createMotionPlanRequest(std::vector<geometry_msgs::Pose> &poses)
{
    moveit::planning_interface::MoveGroupInterface move_group(config_.group_name);
    move_group.setPlannerId(config_.planner_id); //RRTConnect
    move_group.setPlanningTime(10.0f);
    move_group.setMaxVelocityScalingFactor(config_.max_joint_speed_scaling_between_traj);

    // setting above-table position as target
    if (!move_group.setNamedTarget(HOME_POSITION_NAME))
    {
        ROS_ERROR_STREAM("Failed to set home '" << HOME_POSITION_NAME << "' position");
        exit(-1);
    }

    moveit_msgs::MoveItErrorCodes result = move_group.move();
    if (result.val != result.SUCCESS)
    {
        ROS_ERROR_STREAM("Failed to move to " << HOME_POSITION_NAME << " position");
        exit(-1);
    }
    else
    {
        ROS_INFO_STREAM("Robot reached home position");
    }

    moveit::planning_interface::MoveGroupInterface::Plan initial_point_plan;
    std::vector<geometry_msgs::Pose> initial_pose;
    initial_pose.emplace_back(poses[0]);
    moveit_msgs::RobotTrajectory initial_point_trajectory;

    double initial_jump_threshold = 0.00;
    double initial_eef_step = 0.1;
    double initial_fraction = move_group.computeCartesianPath(initial_pose, initial_eef_step, initial_jump_threshold, initial_point_trajectory);

    // addTimeParameterizationToOmpl(initial_point_trajectory, config_.ee_speed, 1);
    initial_point_plan.trajectory_ = initial_point_trajectory;

    if (move_group.execute(initial_point_plan))
    {
        ROS_INFO_STREAM("Robot path execution completed");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to run robot path with error ");
        exit(-1);
    }
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    double jump_threshold = 0.00;
    double eef_step = config_.distance_waypoints;  // If the distance between points has decimal numbers it is required to add that. ex: 0.02038125;
    double fraction = move_group.computeCartesianPath(poses, eef_step, jump_threshold, trajectory);

    addTimeParameterizationToOmpl(trajectory, config_.ee_speed, 1);
    my_plan.trajectory_ = trajectory;

    if (move_group.execute(my_plan))
    {
        ROS_INFO_STREAM("Robot path execution completed");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to run robot path with error ");
        exit(-1);
    }

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

void LayLaOMPLMoveitPlanner::addTimeParameterizationToOmpl(moveit_msgs::RobotTrajectory &traj, double ee_speed, double max_speed_scaling)
{
    robot_trajectory::RobotTrajectory robot_trajectory(robot_model_loader_->getModel(), config_.group_name);

    for (size_t i = 0; i < traj.joint_trajectory.points.size() -1; i++)
    {
        traj.joint_trajectory.points[i].velocities.clear();
        traj.joint_trajectory.points[i].accelerations.clear();
        traj.joint_trajectory.points[i].effort.clear();
    }

    robot_trajectory.setRobotTrajectoryMsg(*kinematic_state_, traj);
    //time_parameterization_.computeTimeStamps(robot_trajectory, 0.05, 1);

    layla_motion_planning::ConstEESpeedTimeParameterization designed_time_parameterization;
    designed_time_parameterization.computeTimeStamps(robot_trajectory, config_.tip_link, ee_speed, max_speed_scaling, 1);

    robot_trajectory.getRobotTrajectoryMsg(traj);
}

} // namespace layla_motion_planning

// void LayLaOMPLMoveitPlanner::loadRobotModel()
// {
//     robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));

//     planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_));

//     /* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
//                      the internal planning scene accordingly */
//     planning_scene_monitor_->startSceneMonitor();
//     /* listens to changes of world geometry, collision objects, and (optionally) octomaps
//                               world geometry, collision objects and optionally octomaps */
//     planning_scene_monitor_->startWorldGeometryMonitor();
//     /* listen to joint state updates as well as changes in attached collision objects
//                       and update the internal planning scene accordingly*/
//     planning_scene_monitor_->startStateMonitor();

//     kinematic_model_ = robot_model_loader_->getModel();

//     if (!kinematic_model_)
//     {
//         ROS_ERROR_STREAM("Failed to load robot model from robot description parameter:robot_description");
//         exit(-1);
//     }

//     kinematic_state_.reset(new robot_state::RobotState(planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState()));

//     joint_model_group_ = kinematic_state_->getJointModelGroup(config_.group_name);

//     // Create pipeline
//     planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(kinematic_model_, nh_, "planning_plugin", "request_adapters"));
// }

// void LayLaOMPLMoveitPlanner::createMotionPlanRequest(geometry_msgs::PoseArray &poses)
// {
//     const std::vector<std::string> &joint_names = joint_model_group_->getActiveJointModelNames();
//     const std::vector<std::string> &link_model_names = joint_model_group_->getLinkModelNames();
//     ROS_INFO_NAMED(NODE_NAME, "end effector name %s\n", link_model_names.back().c_str());

//     planning_interface::MotionPlanRequest req;
//     planning_interface::MotionPlanResponse res;

//     req.group_name = config_.group_name;

//     kinematic_state_::robotStateToRobotStateMsg(*kinematic_state_, req.start_state);
//     poses.header.frame_id = config_.group_name;
//     std::vector<double> goal_joint_values = {0.8, 0.7, 1, -1.3, 1.9, 2.2, -0.1};
//     robot_state->setJointGroupPositions(joint_model_group, goal_joint_values);
//     robot_state->update();
//     moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(*robot_state, joint_model_group);
//     req.goal_constraints.push_back(joint_goal);

//     // Set joint tolerance
//     std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = req.goal_constraints[0].joint_constraints;
//     for (std::size_t x = 0; x < goal_joint_constraint.size(); ++x)
//     {
//         ROS_INFO_STREAM_NAMED(NODE_NAME, " ======================================= joint position at goal: " << goal_joint_constraint[x].position);
//         req.goal_constraints[0].joint_constraints[x].tolerance_above = 0.001;
//         req.goal_constraints[0].joint_constraints[x].tolerance_below = 0.001;
//     }

//     geometry_msgs::Pose pose_msg_goal;
//     const Eigen::Isometry3d &end_effector_transform_goal = robot_state->getGlobalLinkTransform(link_model_names.back());
//     pose_msg_goal = tf2::toMsg(end_effector_transform_goal);

//     // Solve the problem
//     // ========================================================================================
//     // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
//     // representation while planning
//     {
//         planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
//         /* Now, call the pipeline and check whether planning was successful. */
//         planning_pipeline->generatePlan(lscene, req, res);
//     }
//     /* Check that the planning was successful */
//     if (res.error_code_.val != res.error_code_.SUCCESS)
//     {
//         ROS_ERROR_STREAM_NAMED(NODE_NAME, "Could not compute plan successfully");
//         return 0;
//     }

// }

// // void LayLaOMPLMoveitPlanner::loadRobotModel()
// // {
// //     /* First, set the state in the planning scene to the final state of the last plan */

// //     kinematic_state_ = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentStateUpdated(response.trajectory_start);
// //     kinematic_state_->setJointGroupPositions(joint_model_group_, response.trajectory.joint_trajectory.points.back().positions);
// //     kinematic_state_::robotStateToRobotStateMsg(*kinematic_state_, req.start_state);

// //     robot_state::RobotState goal_state(*kinematic_state_);

// //     std::vector<double> joint_values = {-1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0};
// //     goal_state.setJointGroupPositions(joint_model_group, joint_values);
// //     moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

// //     req.goal_constraints.clear();
// //     req.goal_constraints.push_back(joint_goal);

// // }

// } // namespace layla_motion_planning