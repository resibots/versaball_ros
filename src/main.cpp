#include <versaball/versaball.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "versaball");

  versaball::VersaballNode gripper;
  if (gripper.advertise_services())
  {
      ROS_INFO("Spinning node");
      ros::spin();
  }
  else
  {
      ROS_ERROR_STREAM("Failed to advertise services; quitting.");
  }
  return 0;
}