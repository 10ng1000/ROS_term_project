#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>


namespace smb_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

private:
	bool readParameters();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void pillarVisualization();

	ros::NodeHandle nodeHandle_;
	ros::Subscriber scansubscriber_;
	std::string scanTopic_;
	ros::Publisher cmdvelpublisher_;
	ros::Publisher markerpublisher_;
	int queueSize_;

	float xpos;
	float ypos;
};

} /* namespace */
