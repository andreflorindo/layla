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

LayLaTrajoptPlanner::LayLaTrajoptPlanner() : tesseract_(std::make_shared<tesseract::Tesseract>()) {}
LayLaTrajoptPlanner::~LayLaTrajoptPlanner() {}

void LayLaTrajoptPlanner::initRos()
{
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");

  if (ph.getParam("group_name", config_.group_name) &&
      ph.getParam("tip_link", config_.tip_link) &&
      ph.getParam("base_link", config_.base_link) &&
      ph.getParam("world_frame", config_.world_frame) &&
      ph.getParam("layer", config_.layer) &&
      ph.getParam("course", config_.course) &&
      ph.getParam("distance_waypoints", config_.distance_waypoints) &&
      ph.getParam("ee_speed", config_.ee_speed) &&
      ph.getParam("max_velocity_scaling", config_.max_velocity_scaling) &&
      ph.getParam("plotting", plotting_) &&
      ph.getParam("rviz", rviz_))
  // nh.getParam("controller_joint_names", config_.joint_names))
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

  loadRobotModel();

  ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

void LayLaTrajoptPlanner::loadRobotModel()
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

tesseract_common::VectorIsometry3d LayLaTrajoptPlanner::getCourse()
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

  tesseract_common::VectorIsometry3d poses;
  Eigen::Isometry3d single_pose;
  poses.reserve(srv.response.single_course_poses.poses.size());

  // Modify the single_pose type from PoseArray to Isometry3d
  for (unsigned int i = 0; i < srv.response.single_course_poses.poses.size(); i++)
  {
    tf::poseMsgToEigen(srv.response.single_course_poses.poses[i], single_pose);
    poses.emplace_back(single_pose);
  }

  course_publisher_.publish(srv.response.single_course_poses); 

  ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");

  return poses;
}

ProblemConstructionInfo LayLaTrajoptPlanner::trajoptPCI()
{
  ProblemConstructionInfo pci(tesseract_);

  tesseract_common::VectorIsometry3d tool_poses = getCourse();

  // Populate Basic Info
  pci.basic_info.n_steps = static_cast<int>(tool_poses.size());
  pci.basic_info.manip = "manipulator";
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = false;
  pci.opt_info.max_iter = 200;
  pci.opt_info.min_approx_improve = 1e-3;
  pci.opt_info.min_trust_box_size = 1e-3;

  // Create Kinematic Object
  pci.kin = pci.getManipulator(pci.basic_info.manip);

  // Get Init Pose
  EnvState::ConstPtr current_state = pci.env->getCurrentState();
  Eigen::VectorXd start_pos;
  start_pos.resize(pci.kin->numJoints());
  int cnt = 0;
  for (const auto &j : pci.kin->getJointNames())
  {
    start_pos[cnt] = current_state->joints.at(j);
    ++cnt;
  }

  // Repeats initial position given in the code n times, being that the only point of the initial trajectory
  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);
  //pci.init_info.data = readInitTraj("/examples/descartes_sparse_50Hz_sim_joint_request.txt");

  // Populate Cost Info
  std::shared_ptr<JointVelTermInfo> joint_vel = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  joint_vel->coeffs = std::vector<double>(6, 1.0); // 7 double with value 1
  joint_vel->targets = std::vector<double>(6, 0.0);
  // std::vector<double> jv_lower_lim{-0.1, -0.1, -0.1, -0.1, -0.1, -0.1};
  // std::vector<double> jv_upper_lim{0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  // joint_vel->lower_tols = jv_lower_lim;
  // joint_vel->upper_tols = jv_upper_lim;
  joint_vel->first_step = 0;
  joint_vel->last_step = pci.basic_info.n_steps - 1;
  joint_vel->name = "joint_vel";
  joint_vel->term_type = TT_COST;
  pci.cost_infos.push_back(joint_vel);

  std::shared_ptr<JointAccTermInfo> joint_acc = std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
  joint_acc->coeffs = std::vector<double>(6, 2.0);
  joint_acc->targets = std::vector<double>(6, 0.0);
  joint_acc->first_step = 0;
  joint_acc->last_step = pci.basic_info.n_steps - 1;
  joint_acc->name = "joint_acc";
  joint_acc->term_type = TT_COST;
  pci.cost_infos.push_back(joint_acc);

  std::shared_ptr<JointJerkTermInfo> joint_jerk = std::shared_ptr<JointJerkTermInfo>(new JointJerkTermInfo);
  joint_jerk->coeffs = std::vector<double>(6, 5.0);
  joint_jerk->targets = std::vector<double>(6, 0.0);
  joint_jerk->first_step = 0;
  joint_jerk->last_step = pci.basic_info.n_steps - 1;
  joint_jerk->name = "joint_jerk";
  joint_jerk->term_type = TT_COST;
  pci.cost_infos.push_back(joint_jerk);

  std::shared_ptr<CollisionTermInfo> collision = std::shared_ptr<CollisionTermInfo>(new CollisionTermInfo);
  collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 20);
  collision->name = "collision";
  collision->term_type = TT_COST;
  collision->evaluator_type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 1;
  pci.cost_infos.push_back(collision);

  // Populate Constraints

  // std::shared_ptr<CartVelTermInfo> ee_speed = std::shared_ptr<CartVelTermInfo>(new CartVelTermInfo);
  // ee_speed->link = config_.tip_link;
  // ee_speed->term_type = TT_CNT;
  // ee_speed->first_step = 0;
  // ee_speed->max_displacement = 0.007; //0.574/77
  // ee_speed->last_step = pci.basic_info.n_steps - 1;
  // ee_speed->name = "cart_velocity_cnt";
  // pci.cnt_infos.push_back(ee_speed);

  // std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);
  // std::vector<double> vel_lower_lim{-2.14, -2.00, -1.95, -3.12, -3.00, -3.82};
  // std::vector<double> vel_upper_lim{2.14, 2.00, 1.95, 3.12, 3.00, 3.82};
  // jv->coeffs = std::vector<double>(6, 50.0);
  // jv->targets = std::vector<double>(6, 0.0);
  // jv->lower_tols = vel_lower_lim;
  // jv->upper_tols = vel_upper_lim;
  // jv->term_type = TT_CNT;
  // jv->first_step = 0;
  // jv->last_step = pci.basic_info.n_steps - 1;
  // jv->name = "joint_velocity_cnt";
  // pci.cnt_infos.push_back(jv);

  for (auto i = 0; i < pci.basic_info.n_steps; ++i)
  {
    Eigen::Isometry3d current_pose = tool_poses[static_cast<unsigned long>(i)];
    Eigen::Quaterniond q(current_pose.linear());
    Eigen::Vector3d current_xyz = current_pose.translation();
    Eigen::Vector4d current_wxyz = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());

    std::shared_ptr<CartPoseTermInfo> pose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
    pose->term_type = TT_CNT;
    pose->name = "waypoint_cart_" + std::to_string(i);
    pose->link = config_.tip_link;
    // pose->tcp = current_pose;
    pose->timestep = i;
    pose->xyz = current_xyz;
    pose->wxyz = current_wxyz;
    pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
    pose->rot_coeffs = Eigen::Vector3d(10, 10, 10); //With 10 in the latter to be fully constraint

    pci.cnt_infos.push_back(pose);
  }
  return pci;
}

trajectory_msgs::JointTrajectory LayLaTrajoptPlanner::trajArrayToJointTrajectoryMsg(std::vector<std::string> joint_names,
                                                                                  TrajArray traj_array,
                                                                                  bool use_time,
                                                                                  ros::Duration time_increment)
{
  // Create the joint trajectory
  trajectory_msgs::JointTrajectory traj_msg;
  traj_msg.header.stamp = ros::Time::now();
  traj_msg.header.frame_id = "0";
  traj_msg.joint_names = joint_names;

  TrajArray pos_mat;
  TrajArray time_mat;
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

  //addVel(traj_msg);
  // addAcc(traj_msg);

  return traj_msg;
}


void LayLaTrajoptPlanner::addVel(trajectory_msgs::JointTrajectory &traj) //Velocity of the joints
{
  if (traj.points.size() < 3)
    return;

  auto n_joints = traj.points.front().positions.size();

  for (auto i = 0; i < n_joints; ++i)
  {
    traj.points[0].velocities[i] = 0.0f;
    traj.points[traj.points.size() - 1].velocities[i] = 0.0f;
    for (auto j = 1; j < traj.points.size() - 1; j++)
    {
      // For each point in a given joint
      //Finite difference, first order, central. Gives the average velocity, not conservative
      double delta_theta = -traj.points[j - 1].positions[i] + traj.points[j + 1].positions[i];
      double delta_time = -traj.points[j - 1].time_from_start.toSec() + traj.points[j + 1].time_from_start.toSec();
      double v = delta_theta / delta_time;
      traj.points[j].velocities[i] = v;
    }
  }
}

void LayLaTrajoptPlanner::addAcc(trajectory_msgs::JointTrajectory &traj) //Velocity of the joints
{
    if (traj.points.size() < 3)
        return;

    auto n_joints = traj.points.front().positions.size();

    for (auto i = 0; i < n_joints; ++i)
    {
        traj.points[0].accelerations[i] = 0.0f;      // <- Incorrect!!!! TODO
        traj.points[traj.points.size() - 1].accelerations[i] = 0.0f;

        for (auto j = 1; j < traj.points.size()-1; j++)
        {
            // For each point in a given joint
            //Finite difference, first order, central. Gives the average velocity, not conservative
            double delta_velocity = -traj.points[j - 1].velocities[i] + traj.points[j + 1].velocities[i];
            double delta_time = -traj.points[j - 1].time_from_start.toSec() + traj.points[j + 1].time_from_start.toSec();
            double a = delta_velocity / delta_time;
            traj.points[j].accelerations[i] = a;
        }
    }
}

TrajArray LayLaTrajoptPlanner::readInitTraj(std::string start_filename)
{
  std::string filename = ros::package::getPath("layla_msgs") + start_filename;
  std::ifstream infile{filename, std::ios::in};

  if (!infile.good())
  {
    ROS_ERROR_STREAM("Init trajectory has not able to be found. Trajectory generation failed");
    exit(-1);
  }

  std::istream_iterator<double> infile_begin{infile};
  std::istream_iterator<double> eof{};
  std::vector<double> file_nums{infile_begin, eof};
  infile.close();

  int npoints = file_nums.size() / 6;

  TrajArray init_trajectory(npoints, 6);
  int t = 0;

  for (int row = 0; row < npoints; row++)
  {
    for (int col = 0; col < 6; col++)
    {
      // ROS_INFO_STREAM(row);
      // ROS_INFO_STREAM(col);
      // ROS_INFO_STREAM(file_nums[t]);
      init_trajectory(row, col) = file_nums[t];
      t++;
    }
  }
  // ROS_INFO_STREAM(init_trajectory.matrix());
  ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
  return init_trajectory;
}

} // namespace layla_motion_planner
