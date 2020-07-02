#include <layla_descartes_tesseract_planner.h>

namespace layla_motion_planner
{
bool LayLaDescartesTesseractPlanner::run()
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

    tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    {
        ROS_ERROR_STREAM("Tesseract failed to connect to the robot files");
        return false;
    }
    // Create plotting tool
    tesseract_rosutils::ROSPlottingPtr plotter =
        std::make_shared<tesseract_rosutils::ROSPlotting>(tesseract_->getEnvironment());

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

    ROS_INFO_STREAM("ROS SETUP completed");

    ///////////////////
    /// ROBOT SETUP ///
    ///////////////////

    opw_kinematics::Parameters<double> opw_params;
    opw_params.a1 = (0.350);
    opw_params.a2 = (0.041);
    opw_params.b = (0.000);
    opw_params.c1 = (0.675);
    opw_params.c2 = (1.150);
    opw_params.c3 = (1.200);
    opw_params.c4 = (0.215);
    opw_params.offsets[1] = M_PI / 2.0;
    opw_params.sign_corrections[0] = -1;
    opw_params.sign_corrections[3] = -1;
    opw_params.sign_corrections[5] = -1;

    tesseract_kinematics::ForwardKinematics::Ptr robot_kin_fk = tesseract_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
    std::shared_ptr<tesseract_kinematics::OPWInvKin> opw_kin = std::make_shared<tesseract_kinematics::OPWInvKin>();
    opw_kin->init("manipulator",
                  opw_params,
                  robot_kin_fk->getBaseLinkName(),
                  robot_kin_fk->getTipLinkName(),
                  robot_kin_fk->getJointNames(),
                  robot_kin_fk->getLinkNames(),
                  robot_kin_fk->getActiveLinkNames(),
                  robot_kin_fk->getLimits());

    tesseract_->getInvKinematicsManager()->addInvKinematicSolver(opw_kin);

    // Set the initial state of the robot
    std::unordered_map<std::string, double> initial_joint_states;
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

    ROS_INFO_STREAM("ROBOT SETUP completed");

    /////////////////////////////////////
    /// DESCARTES PROBLEM CONSTRUCTION //
    /////////////////////////////////////

    // These specify the series of points to be optimized
    std::vector<tesseract_motion_planners::Waypoint::Ptr> waypoints = getCourse();

    tesseract_kinematics::InverseKinematics::Ptr robot_kin_ik = tesseract_->getInvKinematicsManagerConst()->getInvKinematicSolver("manipulator", "OPWInvKin");
    tesseract_environment::EnvState::ConstPtr current_state = tesseract_->getEnvironmentConst()->getCurrentState();

    tesseract_motion_planners::DescartesMotionPlannerConfigD config = createDescartesPlannerConfig(
        tesseract_, "manipulator", robot_kin_ik, 2.7, current_state, waypoints, false);

    // config.num_threads = 1;

    tesseract_motion_planners::DescartesMotionPlanner<double> single_descartes_planner;
    tesseract_motion_planners::PlannerResponse single_planner_response;
    single_descartes_planner.setConfiguration(config);
    tesseract_common::StatusCode single_status = single_descartes_planner.solve(single_planner_response, true);
    ROS_INFO("Trajectory is planned");

    // tesseract_planning::DescartesTrajOptArrayPlanner descartes_trajopt_planner;

    // descartes_trajopt_planner.setConfiguration(descartes_config, trajopt_config);

    // // ///////////////////////
    // // /// FINAL TRAJECTORY //
    // // ///////////////////////

    if (plotting_)
        plotter->clear();

    plotter->plotTrajectory(robot_kin_ik->getJointNames(), single_planner_response.joint_trajectory.trajectory.leftCols(robot_kin_ik->getJointNames().size()));

    // //////////////
    // /// EXECUTE //
    // //////////////

    trajectory_msgs::JointTrajectory traj_msg;
    ros::Duration t1(0.10);
    traj_msg = trajArrayToJointTrajectoryMsg(robot_kin_ik->getJointNames(), single_planner_response.joint_trajectory.trajectory, false, t1);

    // Create action message
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj_msg;

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
} // namespace layla_motion_planner