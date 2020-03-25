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
	ros::Publisher servo_pub_;
    ros::Publisher drive_pub_;
	ros::Subscriber joy_sub_;
	ros::NodeHandle nodeHandle_;
};

} /* namespace */
