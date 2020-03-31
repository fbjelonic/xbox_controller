#pragma once

#include <ros/ros.h>
#include "std_msgs/UInt8.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
// #include "xbox_controller/Engine.h"

namespace xbox_controller {

class XboxController {
public:
  XboxController(ros::NodeHandle& nodeHandle);
  virtual ~XboxController();
  void joyCallback(const sensor_msgs::Joy& msg);

private:
  bool readParameters();

  void handleUltrasonicSensor(double controllerInput);
  void handleDiffdrive(double rt, double lt, double left_joy);
  void sendRequest();
  int16_t threshold(double wheel);

  int right_joystick_;
  int left_joystick_;
  int rt_button_;
  int lt_button_;
  int max_vel_;

  std_msgs::Int16 left_wheel_;
  std_msgs::Int16 right_wheel_;
  std_msgs::UInt8 servo_angle_;

  ros::Publisher raspi_pub_;
  ros::Subscriber joy_sub_;
  ros::NodeHandle nodeHandle_;
};

}
