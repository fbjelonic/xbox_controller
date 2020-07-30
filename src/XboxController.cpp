#include "xbox_controller/XboxController.hpp"

namespace xbox_controller {

XboxController::XboxController(ros::NodeHandle& nodeHandle) :
  lt_used_(false), rt_used_(false), nodeHandle_(nodeHandle), it_(nodeHandle_)
{
  //cv::VideoCapture cap;
  //cap.open(0);
  //cap.read(image_);

  //out_msg_->image = image_;
  //out_msg_->header.stamp = ros::Time::now();
  //out_msg_->header.frame_id = "world";
  //out_msg_->header.seq = 24;
  //out_msg_->encoding = sensor_msgs::image_encodings::BGR8;

  // publish messages for raspberry pi (for 2wd arduino)
  raspi_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("engine", 10);

  // get controller input from joy topic
  joy_sub_ = nodeHandle_.subscribe("/joy", 10, &XboxController::joyCallback, this);

  // publish image to yolo_ros
  image_sub_ = nodeHandle_.subscribe("/darknet_ros/detection_image", 1, &XboxController::imageCallback, this);
  //image_pub_ = it_.advertise("/camera/rgb/image_raw", 1);
  ROS_WARN("Sending mesage now:");
  ROS_WARN("Finished sending messages");

  servo_angle_.data = 90;
  left_wheel_.data = 0;
  right_wheel_.data = 0;

  if (!readParameters())
  {
    ROS_ERROR("Could not read from parameter server!");
    ros::shutdown();
  }
  max_vel_  = max_vel_ * ratio_;
}



bool XboxController::readParameters()
{
  if (!nodeHandle_.getParam("right_joystick", right_joystick_)) return false;
  if (!nodeHandle_.getParam("left_joystick", left_joystick_)) return false;
  if (!nodeHandle_.getParam("rt_button", rt_button_)) return false;
  if (!nodeHandle_.getParam("lt_button", lt_button_)) return false;
  if (!nodeHandle_.getParam("max_vel", max_vel_)) return false;
  if (!nodeHandle_.getParam("speed_ratio", ratio_)) return false;
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
  //cv::imshow("Live", image_);
  //cv::waitKey(0);
  //image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, image_).toImageMsg());
  // extract value of xbox controller
  double rightJoystickValue = static_cast<double>(msg.axes.at(static_cast<size_t>(right_joystick_)));
  double leftJoystickValue = static_cast<double>(msg.axes.at(static_cast<size_t>(left_joystick_)));
  double rt_buttonValue = static_cast<double>(msg.axes.at(static_cast<size_t>(rt_button_)));
  double lt_buttonValue = static_cast<double>(msg.axes.at(static_cast<size_t>(lt_button_)));

  handleUltrasonicSensor(rightJoystickValue);
  handleDiffdrive(rt_buttonValue, lt_buttonValue, leftJoystickValue);

  sendRequest();
}

void XboxController::imageCallback(const sensor_msgs::Image &img)
{
  cv_bridge::CvImagePtr ptr;
  ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  cv::imshow("Live", ptr->image);
  cv::waitKey(0);
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

  if (rt!=0 && !rt_used_)
  {
    rt_used_ = true;
  }
  if (lt!=0 && !lt_used_)
  {
    lt_used_ = true;
  }

  // wheel velocity
  if (rt < 1 && rt_used_) {
    left_wheel = - (rt-1) * static_cast<double>(max_vel_) / 2; // forward
    right_wheel = - (rt-1) * static_cast<double>(max_vel_) / 2;
  }
  else if (lt < 1 && lt_used_) {
    left_wheel = (lt-1) * static_cast<double>(max_vel_) / 2; // forward
    right_wheel = (lt-1) * static_cast<double>(max_vel_) / 2;
  }

  left_wheel -= angular_speed;
  right_wheel += angular_speed;
  ROS_INFO("Left wheel: %f, right wheel: %f", left_wheel, right_wheel);

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
