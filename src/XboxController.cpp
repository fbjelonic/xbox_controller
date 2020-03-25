#include "xbox_controller/XboxController.hpp"

namespace xbox_controller {

XboxController::XboxController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  servo_pub_ = nodeHandle_.advertise<std_msgs::UInt16>("/servo", 10);
  drive_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("/drive", 10);
  joy_sub_ = nodeHandle_.subscribe("/joy", 10, &XboxController::joyCallback, this);
}

XboxController::~XboxController()
{
}

void XboxController::joyCallback(const sensor_msgs::Joy& msg)
{
    std_msgs::UInt16 newmsg;
    geometry_msgs::Twist drivemsg;
    newmsg.data = 90;
    drivemsg.linear.x = 0;
    drivemsg.angular.z = 0;
  if (msg.axes.at(3) > 0.2 || msg.axes.at(3) < -0.2) {
      newmsg.data = 90 + 90 * msg.axes.at(3);
  }
  if (msg.axes.at(0) > 0.1 || msg.axes.at(0) < -0.1) {
      drivemsg.angular.z = msg.axes.at(0) * 255;
  }
  if (msg.axes.at(2) < 0) {
      drivemsg.linear.x = msg.axes.at(2) * 255;
  }
  if (msg.axes.at(5) < 0) {
      drivemsg.linear.x = - msg.axes.at(5) * 255;
  }
  servo_pub_.publish(newmsg);
  drive_pub_.publish(drivemsg);
}

} /* namespace */
