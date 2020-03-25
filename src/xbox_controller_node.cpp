#include <ros/ros.h>
#include "xbox_controller/XboxController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_controller");
  ros::NodeHandle nodeHandle("~");

  xbox_controller::XboxController xboxcontroller(nodeHandle);

  ros::spin();
  return 0;
}
