/* Author: Andre Florindo*/

// Add vector<CourseStruct> in case there are more courses
// If z is not given in the file, maybe add a collumn of zeros

#include <pose_builder_server.h>

namespace layla_motion_planning
{

PoseBuilder::PoseBuilder() {}
PoseBuilder::~PoseBuilder() {}

void PoseBuilder::initServer()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("world_frame", config_.world_frame))
    {
        ROS_INFO_STREAM("pose_builder: Loaded Server parameters");
    }
    else
    {
        ROS_ERROR_STREAM("pose_builder: Failed to load Server parameters");
        exit(-1);
    }

    ROS_INFO_STREAM("pose_builder: Task '" << __FUNCTION__ << "' completed");
}

void PoseBuilder::createCourse()
{
    //Read File containing the course
    CourseStruct course;
    //std::shared_ptr<CourseStruct> course = std::make_shared<CourseStruct>();
    readFileContent("/examples/simplePath.txt", course);
    int npoints = course.x.size();

    //Read Files with the binormal and tangent of the course
    CourseStruct tangent;
    CourseStruct binormal;
    readFileContent("/examples/tangent_simplePath.txt", tangent);
    readFileContent("/examples/binormal_simplePath.txt", binormal);

    //Initializate pose message
    course_poses.poses.reserve(npoints); //  <---------------
    course_poses.header.frame_id = config_.world_frame;

    //determining orientation and calculate pose
    Eigen::Vector3d ee_z, ee_y, ee_x;
    Eigen::Isometry3d single_pose;
    geometry_msgs::Pose single_pose_msg;

    for (int i = 0; i < npoints; i++)
    {
        ee_z << -binormal.x[i], -binormal.y[i], -binormal.z[i];
        ee_x << -tangent.x[i], -tangent.y[i], -tangent.z[i];
        ee_y = (ee_z.cross(ee_x)).normalized();

        Eigen::Isometry3d rot;
        rot.matrix() << ee_x(0), ee_y(0), ee_z(0), 0, ee_x(1), ee_y(1), ee_z(1), 0, ee_x(2), ee_y(2), ee_z(2), 0, 0, 0, 0, 1;
        //single_pose = Eigen::Translation3d(course.x[i]-0.8, course.y[i]+1.6, course.z[i]+0.8) * rot; //-0.8 1.6 0.8
        
        Eigen::Isometry3d rot_start_table;
        rot_start_table.matrix() << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
        // single_pose = rot_start_table * (Eigen::Translation3d(course.x[i], course.y[i] - TABLE_WIDTH - TABLE_WIDTH / 2, course.z[i] + TABLE_HEIGHT + APPROACH_TABLE)) * rot;
        single_pose = rot_start_table * (Eigen::Translation3d(course.x[i]- TABLE_WIDTH - TABLE_WIDTH / 2, course.y[i] - TABLE_WIDTH/2, course.z[i] + TABLE_HEIGHT + APPROACH_TABLE +0.7)) * rot;

        tf::poseEigenToMsg(single_pose, single_pose_msg);
        course_poses.poses.emplace_back(single_pose_msg);
    }

    // introduceSmoothApproximantion(0, tangent, binormal, course);
    // introduceSmoothApproximantion(npoints - 1, tangent, binormal, course);

    pose_builder_server_ = nh_.advertiseService(POSE_BUILDER_SERVICE, &PoseBuilder::serviceCallback, this); //  <---------------

    ROS_INFO_STREAM("pose_builder: Task '" << __FUNCTION__ << "' completed");
    ROS_INFO_STREAM("pose_builder: Trajectory with " << npoints << " points was generated");
}

void PoseBuilder::readFileContent(std::string start_filename, CourseStruct &course)
{
    std::string filename = ros::package::getPath("layla_msgs") + start_filename;
    std::ifstream infile{filename, std::ios::in};

    if (!infile.good())
    {
        ROS_ERROR_STREAM("pose_builder: Path as not able to be found. Trajectory generation failed");
        exit(-1);
    }

    std::istream_iterator<double> infile_begin{infile};
    std::istream_iterator<double> eof{};
    std::vector<double> file_nums{infile_begin, eof};
    infile.close();

    unsigned int nx = 0;
    unsigned int ny = 0;
    int npoints = file_nums.size() / 3;

    course.x.reserve(npoints);
    course.y.reserve(npoints);
    course.z.reserve(npoints);

    for (unsigned int t = 0; t < file_nums.size(); t++)
    {
        if (t == nx * 3)
        {
            course.x.emplace_back(file_nums[t]);
            nx++;
        }
        else if (t == 1 + ny * 3)
        {
            course.y.emplace_back(file_nums[t]);
            ny++;
        }
        else
            course.z.emplace_back(file_nums[t]);
    }
}

void PoseBuilder::introduceSmoothApproximantion(int i, CourseStruct &tangent, CourseStruct &binormal, CourseStruct &course)
{
    Eigen::Vector3d ee_z, ee_y, ee_x;
    Eigen::Isometry3d single_pose;
    geometry_msgs::Pose single_pose_msg;

    ee_z << -binormal.x[i], -binormal.y[i], -binormal.z[i];
    ee_x << -tangent.x[i], -tangent.y[i], -tangent.z[i];
    ee_y = (ee_z.cross(ee_x)).normalized();

    Eigen::Isometry3d rot;
    rot.matrix() << ee_x(0), ee_y(0), ee_z(0), 0, ee_x(1), ee_y(1), ee_z(1), 0, ee_x(2), ee_y(2), ee_z(2), 0, 0, 0, 0, 1;
    Eigen::Isometry3d rot_start_table;
    rot_start_table.matrix() << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    int course_extension_npoints = 2;
    int raise_course_npoints = 3;
    double signal;

    if (i == 0)
    {
        signal = 1.0;
    }
    else
    {
        signal = -1.0;
    }
    
    CourseStruct smooth_course_approximation;
    smooth_course_approximation.x.reserve(course_extension_npoints + raise_course_npoints);
    smooth_course_approximation.y.reserve(course_extension_npoints + raise_course_npoints);
    smooth_course_approximation.z.reserve(course_extension_npoints + raise_course_npoints);

    for (int f = 1; f <= course_extension_npoints; f++)
    {
        smooth_course_approximation.x.emplace_back(course.x[i] - signal * ((double)f / (double)course_extension_npoints) * (tangent.x[i] / (tangent.x[i] + tangent.y[i])) * XY_EXTENSION_DISTANCE);
        smooth_course_approximation.y.emplace_back(course.y[i] - signal * ((double)f / (double)course_extension_npoints) * (tangent.y[i] / (tangent.x[i] + tangent.y[i])) * XY_EXTENSION_DISTANCE);
        smooth_course_approximation.z.emplace_back(course.z[i]);
    }

    for (int f = 1; f <= raise_course_npoints; f++)
    {
        smooth_course_approximation.x.emplace_back(smooth_course_approximation.x[course_extension_npoints-1] - signal*((double)f / (double)raise_course_npoints) * (tangent.x[i] / (tangent.x[i] + tangent.y[i])) * XY_RAISE_DISTANCE);
        smooth_course_approximation.y.emplace_back(smooth_course_approximation.y[course_extension_npoints-1] - signal*((double)f / (double)raise_course_npoints) * (tangent.y[i] / (tangent.x[i] + tangent.y[i])) * XY_RAISE_DISTANCE);
        smooth_course_approximation.z.emplace_back(smooth_course_approximation.z[course_extension_npoints-1] + ((double)f / (double)raise_course_npoints) * Z_RAISE_DISTANCE);
    }

    for (int f = 0; f < course_extension_npoints + raise_course_npoints; f++)
    {
        single_pose = rot_start_table * (Eigen::Translation3d(smooth_course_approximation.x[f], smooth_course_approximation.y[f] - TABLE_WIDTH - TABLE_WIDTH / 2, smooth_course_approximation.z[f] + TABLE_HEIGHT + APPROACH_TABLE)) * rot;
        tf::poseEigenToMsg(single_pose, single_pose_msg);
        if (i == 0)
        {
            course_poses.poses.insert(course_poses.poses.begin(), single_pose_msg);
        }
        else
        {
            course_poses.poses.insert(course_poses.poses.end(), single_pose_msg);
        }
    }
}

bool PoseBuilder::serviceCallback(layla_msgs::PoseBuilder::Request &request, layla_msgs::PoseBuilder::Response &response)
{
    response.single_course_poses = course_poses; //  <---------------
    return true;
}

} // namespace layla_motion_planning

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_builder");

    layla_motion_planning::PoseBuilder pose_builder;

    pose_builder.initServer();

    pose_builder.createCourse();

    ros::spin();
}


// READ FILE IN C++
// tesseract_common::VectorIsometry3d PuzzlePieceExample::makePuzzleToolPoses()
// {
//   tesseract_common::VectorIsometry3d path; // results
//   std::ifstream indata;                    // input file

//   // You could load your parts from anywhere, but we are transporting them with
//   // the git repo
//   std::string filename = ros::package::getPath("tesseract_ros_examples") + "/config/puzzle_bent.csv";

//   // In a non-trivial app, you'll of course want to check that calls like 'open'
//   // succeeded
//   indata.open(filename);

//   std::string line;
//   int lnum = 0;
//   while (std::getline(indata, line))
//   {
//     ++lnum;
//     if (lnum < 3)
//       continue;

//     std::stringstream lineStream(line);
//     std::string cell;
//     Eigen::Matrix<double, 6, 1> xyzijk;
//     int i = -2;
//     while (std::getline(lineStream, cell, ','))
//     {
//       ++i;
//       if (i == -1)
//         continue;

//       xyzijk(i) = std::stod(cell);
//     }

//     Eigen::Vector3d pos = xyzijk.head<3>();
//     pos = pos / 1000.0; // Most things in ROS use meters as the unit of length.
//                         // Our part was exported in mm.
//     Eigen::Vector3d norm = xyzijk.tail<3>();
//     norm.normalize();

//     // This code computes two extra directions to turn the normal direction into
//     // a full defined frame. Descartes
//     // will search around this frame for extra poses, so the exact values do not
//     // matter as long they are valid.
//     Eigen::Vector3d temp_x = (-1 * pos).normalized();
//     Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
//     Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();
//     Eigen::Isometry3d pose;
//     pose.matrix().col(0).head<3>() = x_axis;
//     pose.matrix().col(1).head<3>() = y_axis;
//     pose.matrix().col(2).head<3>() = norm;
//     pose.matrix().col(3).head<3>() = pos;

//     path.push_back(pose);
//   }
//   indata.close();

//   return path;
// }