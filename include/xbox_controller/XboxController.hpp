#pragma once

#include <ros/ros.h>
#include "std_msgs/UInt8.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <boost/shared_ptr.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
// #include "xbox_controller/Engine.h"

namespace xbox_controller {

class XboxController {
public:
  XboxController(ros::NodeHandle& nodeHandle);
  virtual ~XboxController();
  void joyCallback(const sensor_msgs::Joy& msg);
  void imageCallback(const sensor_msgs::Image& img);

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
  double ratio_;

  bool lt_used_;
  bool rt_used_;

  cv::Mat image_;

  std_msgs::Int16 left_wheel_;
  std_msgs::Int16 right_wheel_;
  std_msgs::UInt8 servo_angle_;

  ros::Publisher raspi_pub_;
  ros::Subscriber joy_sub_;
  ros::NodeHandle nodeHandle_;
  image_transport::ImageTransport it_;
  ros::Subscriber image_sub_;
};

}
