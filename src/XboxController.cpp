#include "xbox_controller/XboxController.hpp"

namespace xbox_controller {

XboxController::XboxController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  // publish messages for raspberry pi (for 2wd arduino)
  raspi_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("engine", 10);

  // get controller input from joy topic
  joy_sub_ = nodeHandle_.subscribe("/joy", 10, &XboxController::joyCallback, this);

  servo_angle_.data = 90;
  left_wheel_.data = 0;
  right_wheel_.data = 0;

  if (!readParameters())
  {
    ROS_ERROR("Could not read from parameter server!");
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
  // Stop the car
  left_wheel_.data = 0;
  right_wheel_.data = 0;
  servo_angle_.data = 90;
  sendRequest();
}

void XboxController::joyCallback(const sensor_msgs::Joy& msg)
{
  // extract value of xbox controller
  double rightJoystickValue = static_cast<double>(msg.axes.at(static_cast<size_t>(right_joystick_)));
  double leftJoystickValue = static_cast<double>(msg.axes.at(static_cast<size_t>(left_joystick_)));
  double rt_buttonValue = static_cast<double>(msg.axes.at(static_cast<size_t>(rt_button_)));
  double lt_buttonValue = static_cast<double>(msg.axes.at(static_cast<size_t>(lt_button_)));

  handleUltrasonicSensor(rightJoystickValue);
  handleDiffdrive(rt_buttonValue, lt_buttonValue, leftJoystickValue);

  sendRequest();
}

void XboxController::sendRequest()
{
  geometry_msgs::Twist my_msg;
  my_msg.linear.x = left_wheel_.data; // left wheel speed
  my_msg.linear.y = right_wheel_.data; // right wheel speed
  my_msg.angular.x = servo_angle_.data; // Ultrasonic Sensor Angle

  raspi_pub_.publish(my_msg);
}

void XboxController::handleUltrasonicSensor(double controllerInput)
{
  if (controllerInput > 0.1 || controllerInput < -0.1) {
    servo_angle_.data = static_cast<uint8_t>(90 + 90 * controllerInput);
  }
  else {
    servo_angle_.data = 90;
  }
}

void XboxController::handleDiffdrive(double rt, double lt, double left_joy)
{
  double angular_speed = 0;
  double left_wheel = 0;
  double right_wheel = 0;
  // angular velocity
  if (left_joy > 0.1 || left_joy < -0.1) {
    angular_speed = left_joy * max_vel_;
  }

  // wheel velocity
  if (rt < 0) {
    left_wheel = - rt * static_cast<double>(max_vel_) - angular_speed; // forward
    right_wheel = - rt * static_cast<double>(max_vel_) + angular_speed;
  }
  else if (lt < 0) {
    left_wheel = - lt * static_cast<double>(max_vel_) - angular_speed; // forward
    right_wheel = - lt * static_cast<double>(max_vel_) + angular_speed;
  }

  left_wheel_.data = threshold(left_wheel);
  right_wheel_.data = threshold(right_wheel);
}

int16_t XboxController::threshold(double wheel)
{
  if (wheel > max_vel_) {
    return static_cast<int16_t>(max_vel_);
  }
  else if (wheel < - max_vel_) {
    return static_cast<int16_t>(-max_vel_);
  }
  return static_cast<int16_t>(wheel);
}

} /* namespace */
