#include <layla_trajopt_planner.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "layla_trajopt_main");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  layla_motion_planner::LayLaTrajoptPlanner planner;
  planner.run();
  
  spinner.stop();
  return 0;
}
