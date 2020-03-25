#include "xbox_controller/XboxController.hpp"

namespace xbox_controller {

XboxController::XboxController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  servo_pub_ = nodeHandle_.advertise<std_msgs::UInt16>("/servo", 10);
  drive_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("/drive", 10);
  joy_sub_ = nodeHandle_.subscribe("/joy", 10, &XboxController::joyCallback, this);
  right_joystick_ = nodeHandle_.param("right_joystick", 3);
  left_joystick_ = nodeHandle_.param("left_joystick", 0);
  rt_button_ = nodeHandle_.param("rt_button", 2);
  lt_button_ = nodeHandle_.param("lt_button", 5);
  max_vel_ = nodeHandle_.param("max_vel", 255);
}

XboxController::~XboxController()
{
    drive_pub_.publish(handleDiffdrive(0,0,0)); // stop the car, in case of execution failure
}

void XboxController::joyCallback(const sensor_msgs::Joy& msg)
{
    double rightJoystick = static_cast<double>(msg.axes.at(static_cast<size_t>(right_joystick_)));

    servo_pub_.publish(handleUltrasonicSensor(rightJoystick));

    double leftJoystick = static_cast<double>(msg.axes.at(static_cast<size_t>(left_joystick_)));
    double rt_button = static_cast<double>(msg.axes.at(static_cast<size_t>(rt_button_)));
    double lt_button = static_cast<double>(msg.axes.at(static_cast<size_t>(lt_button_)));

    drive_pub_.publish(handleDiffdrive(rt_button, lt_button, leftJoystick));
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
