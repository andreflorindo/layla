#include <layla_trajopt_planner.h>

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;

namespace layla_motion_planner
{

bool LayLaTrajoptPlanner::run()
{
    //////////////////
    /// ROS SETUP ///
    /////////////////

    // Pull ROS params
    initRos();

    // Initialize the environment
    std::string urdf_xml_string, srdf_xml_string;
    nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
    nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

    ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    {
        ROS_ERROR_STREAM("Tesseract failed to connect to the robot files");
        return false;
    }
    // Create plotting tool
    tesseract_rosutils::ROSPlottingPtr plotter =
        std::make_shared<tesseract_rosutils::ROSPlotting>(tesseract_->getEnvironment()->getSceneGraph()->getRoot());

    if (rviz_)
    {
        // These are used to keep visualization updated
        modify_env_rviz_ = nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(MODIFY_ENVIRONMENT_SERVICE, false);
        get_env_changes_rviz_ =
            nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(GET_ENVIRONMENT_CHANGES_SERVICE, false);

        // Check RViz to make sure nothing has changed
        if (!checkRviz())
            return false;
    }

    ///////////////////
    /// ROBOT SETUP ///
    ///////////////////

    // Set the initial state of the robot
    std::unordered_map<std::string, double> initial_joint_states;
    // initial_joint_states["kr210_joint_a1"] = 0.3;
    // initial_joint_states["kr210_joint_a2"] = -0.76;
    // initial_joint_states["kr210_joint_a3"] = 1.72;
    // initial_joint_states["kr210_joint_a4"] = 0.0;
    // initial_joint_states["kr210_joint_a5"] = 0.6;
    // initial_joint_states["kr210_joint_a6"] = 0.0;
    initial_joint_states["kr210_joint_a1"] = 1.57;
    initial_joint_states["kr210_joint_a2"] = -1.04;
    initial_joint_states["kr210_joint_a3"] = 1.04;
    initial_joint_states["kr210_joint_a4"] = 0.0;
    initial_joint_states["kr210_joint_a5"] = 1.57;
    initial_joint_states["kr210_joint_a6"] = 0.0;
    tesseract_->getEnvironment()->setState(initial_joint_states);

    if (rviz_)
    {
        // Now update rviz environment
        if (!sendRvizChanges(0))
            return false;
    }

    ///////////////////////////////////
    /// TRAJOPT PROBLEM CONSTRUCTION //
    ///////////////////////////////////

    // Set Log Level
    util::gLogLevel = util::LevelInfo;

    // Setup Problem
    ProblemConstructionInfo pci = trajoptPCI();
    TrajOptProb::Ptr prob = ConstructProblem(pci);

    // Solve Trajectory
    ROS_INFO("Trajectory is planned");

    /////////////////////////
    /// INITIAL TRAJECTORY //
    /////////////////////////

    std::vector<ContactResultMap> collisions;
    tesseract_environment::StateSolver::Ptr state_solver = prob->GetEnv()->getStateSolver();
    ContinuousContactManager::Ptr manager = prob->GetEnv()->getContinuousContactManager();
    AdjacencyMap::Ptr adjacency_map =
        std::make_shared<tesseract_environment::AdjacencyMap>(prob->GetEnv()->getSceneGraph(),
                                                              prob->GetKin()->getActiveLinkNames(),
                                                              prob->GetEnv()->getCurrentState()->link_transforms);

    manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
    manager->setContactDistanceThreshold(0);
    collisions.clear();
    bool found =
        checkTrajectory(collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), prob->GetInitTraj());

    ROS_INFO((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

    ///////////////////
    /// OPTIMIZATION //
    ///////////////////

    sco::BasicTrustRegionSQP opt(prob);
    opt.setParameters(pci.opt_info);
    if (plotting_)
    {
        opt.addCallback(PlotCallback(*prob, plotter));
    }

    ros::Time tStart = ros::Time::now();
    opt.initialize(trajToDblVec(prob->GetInitTraj()));
    sco::OptStatus status = opt.optimize();
    ROS_INFO("Optimization Status: %s, Planning time: %.3f",
             sco::statusToString(status).c_str(),
             (ros::Time::now() - tStart).toSec());

    ///////////////////////
    /// FINAL TRAJECTORY //
    ///////////////////////

    // getTraj() - To get trajectory of type trajopt::TrajArray

    trajopt::TrajArray poses_matrix = getTraj(opt.x(), prob->GetVars());

    if (plotting_)
        plotter->clear();

    collisions.clear();
    found = checkTrajectory(
        collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), poses_matrix);

    ROS_INFO((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));

    // plotter->plotTrajectory(prob->GetKin()->getJointNames(), poses_matrix);

    trajectory_msgs::JointTrajectory traj_msg;
    ros::Duration delta_time(config_.distance_waypoints / config_.ee_speed);
    traj_msg = trajArrayToJointTrajectoryMsg(prob->GetKin()->getJointNames(), poses_matrix, pci.basic_info.use_time, delta_time);

    moveit_msgs::RobotTrajectory moveit_traj;
    moveit_traj.joint_trajectory = traj_msg;

    addTimeParameterizationToTrajopt(moveit_traj);
    
    ROS_ERROR("Solution found! Hit enter key to move robot");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    //////////////
    /// EXECUTE //
    //////////////

    executeStartMotionwithMoveit(poses_matrix);

    // Create action message
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = moveit_traj.joint_trajectory;

    ROS_INFO_STREAM("Robot path sent for execution");

    if (follow_joint_trajectory_client_->sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO_STREAM("Robot path execution completed");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to run robot path with error " << *follow_joint_trajectory_client_->getResult());
        exit(-1);
    }

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");

    return true;
}

void LayLaTrajoptPlanner::executeStartMotionwithMoveit(trajopt::TrajArray poses_matrix)
{
    moveit::planning_interface::MoveGroupInterface move_group(config_.group_name);
    move_group.setPlannerId(PLANNER_ID); //RRTConnect
    move_group.setPlanningTime(10.0f);
    move_group.setMaxVelocityScalingFactor(config_.max_velocity_scaling);

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

    // creating goal joint pose to start of the path
    auto init_pos_mat = poses_matrix.leftCols(poses_matrix.cols());
    auto mat = init_pos_mat.row(0);
    std::vector<double> start_pose(mat.data(), mat.data() + mat.rows() * mat.cols());

    move_group.setJointValueTarget(start_pose);

    result = move_group.move();
    if (result.val != result.SUCCESS)
    {
        ROS_ERROR_STREAM("Move to start joint pose failed");
        exit(-1);
    }
    else
    {
        ROS_INFO_STREAM("Robot reached start position");
    }
}

void LayLaTrajoptPlanner::addTimeParameterizationToTrajopt(moveit_msgs::RobotTrajectory &traj)
{
  robot_trajectory::RobotTrajectory robot_trajectory(robot_model_loader_->getModel(), config_.group_name);
  for (size_t i = 0; i < traj.joint_trajectory.points.size() -1; i++)
  {
    traj.joint_trajectory.points[i].velocities.clear();
    traj.joint_trajectory.points[i].accelerations.clear();
    traj.joint_trajectory.points[i].effort.clear();
  }

  robot_trajectory.setRobotTrajectoryMsg(*kinematic_state_,traj);
  //time_parameterization_.computeTimeStamps(robot_trajectory, 0.05, 1);
  
  layla_motion_planning::ConstEESpeedTimeParameterization designed_time_parameterization;
  designed_time_parameterization.computeTimeStamps(robot_trajectory, config_.tip_link, config_.ee_speed, 1, 1);

  robot_trajectory.getRobotTrajectoryMsg(traj);
}

} // namespace layla_motion_planner