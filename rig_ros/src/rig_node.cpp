#include "rig/radar/radar_manager.hpp"

int main(int argc, char * argv[])
{
  // Initialize ROS Node
  ros::init(argc, argv, "rig");
  ROS_INFO("Radar Velocity Estimation Started");

  // Create Node Handles to be passed
  ros::NodeHandle pnh("~");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  rig::RadarManager rm(pnh);

  ros::waitForShutdown();
  return 0;
}
