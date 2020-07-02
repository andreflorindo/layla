#include <layla_descartes_tesseract_planner.h>

// There are several bugs, therefore the trajetory created is not the requested. Possible bugs will be described next. 
// The OPW kinematics for some reason only works when opw_params.offsets[1] is +90 and not negative, which should be the correct value.
// In the function getCourse(), the waypoints transformation is againts the link config_.base_link and not world_frame.
// For some reason requires the initial position
// The tcp may not be correct given in createDescartesPlannerConfig

int main(int argc, char **argv)
{
  ros::init(argc, argv, "layla_descartes_tesseract_main");

  layla_motion_planner::LayLaDescartesTesseractPlanner planner;
  planner.run();
  ros::spin();
  return 0;
}
