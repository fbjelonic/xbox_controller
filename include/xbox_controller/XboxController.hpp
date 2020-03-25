#pragma once

#include <ros/ros.h>
#include "std_msgs/UInt16.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

namespace xbox_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class XboxController {
public:
	/*!
	 * Constructor.
	 */
	XboxController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~XboxController();
	void joyCallback(const sensor_msgs::Joy& msg);
private:
    std_msgs::UInt16 handleUltrasonicSensor(double controllerInput);
    geometry_msgs::Twist handleDiffdrive(double rt, double lt, double left_joy);

    int right_joystick_;
    int left_joystick_;
    int rt_button_;
    int lt_button_;
    int max_vel_;
	ros::Publisher servo_pub_;
    ros::Publisher drive_pub_;
	ros::Subscriber joy_sub_;
	ros::NodeHandle nodeHandle_;
};

} /* namespace */
