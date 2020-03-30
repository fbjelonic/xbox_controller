#include "xbox_controller/XboxController.hpp"

namespace xbox_controller {

XboxController::XboxController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  // publish messages for raspberry pi (for 2wd arduino)
  servo_pub_ = nodeHandle_.advertise<std_msgs::UInt16>("servo", 10);
  drive_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("drive", 10);

  // get controller input from joy topic
  joy_sub_ = nodeHandle_.subscribe("/joy", 10, &XboxController::joyCallback, this);

  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters from .yaml file!");
    ros::shutdown();
  }
}

bool XboxController::readParameters()
{
  if (!nodeHandle_.getParam("right_joystick", right_joystick_)) return false;
  if (!nodeHandle_.getParam("left_joystick", left_joystick_)) return false;
  if (!nodeHandle_.getParam("rt_button", rt_button_)) return false;
  if (!nodeHandle_.getParam("lt_button", lt_button_)) return false;
  if (!nodeHandle_.getParam("max_vel", max_vel_)) return false;
  return true;
}

XboxController::~XboxController()
{
  drive_pub_.publish(handleDiffdrive(0,0,0)); // stop the car, in case of execution failure or shutdown
}

void XboxController::joyCallback(const sensor_msgs::Joy& msg)
{
  // extract value of right joystick
  double rightJoystickValue = static_cast<double>(msg.axes.at(static_cast<size_t>(right_joystick_)));
  servo_pub_.publish(handleUltrasonicSensor(rightJoystickValue));

  double leftJoystickValue = static_cast<double>(msg.axes.at(static_cast<size_t>(left_joystick_)));
  double rt_buttonValue = static_cast<double>(msg.axes.at(static_cast<size_t>(rt_button_)));
  double lt_buttonValue = static_cast<double>(msg.axes.at(static_cast<size_t>(lt_button_)));

  drive_pub_.publish(handleDiffdrive(rt_buttonValue, lt_buttonValue, leftJoystickValue));
}

std_msgs::UInt16 XboxController::handleUltrasonicSensor(double controllerInput)
{
  std_msgs::UInt16 newmsg;
  newmsg.data = 90; // set sensor to middle if nothing's happening
  if (controllerInput > 0.2 || controllerInput < -0.2) {
    newmsg.data = 90 + 90 * static_cast<uint16_t>(controllerInput);
  }
  return newmsg;
}

geometry_msgs::Twist XboxController::handleDiffdrive(double rt, double lt, double left_joy)
{
  geometry_msgs::Twist drivemsg;
  drivemsg.linear.x = 0;
  drivemsg.angular.z = 0;

  // angular velocity
  if (left_joy > 0.1 || left_joy < -0.1) {
    drivemsg.angular.z = left_joy * max_vel_;
  }

  // linear velocity
  if (rt < 0) {
    drivemsg.linear.x = - rt * 255; // forward
  }
  else if (lt < 0) {
    drivemsg.linear.x = lt * 255; // backward
  }

  return drivemsg;
}

} /* namespace */
