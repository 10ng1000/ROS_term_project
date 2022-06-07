#include <smb_highlevel_controller/SmbHighlevelController.hpp>
#include <algorithm>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  scansubscriber_ = nodeHandle_.subscribe(scanTopic_, queueSize_, &SmbHighlevelController::scanCallback, this);
  cmdvelpublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  markerpublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

  ROS_INFO("Successfully launched node.");
}

SmbHighlevelController::~SmbHighlevelController()
{
}

bool SmbHighlevelController::readParameters()
{
  {
  if (!nodeHandle_.getParam("subscriber_topic", scanTopic_) || !nodeHandle_.getParam("queue_size", queueSize_)) return false;
  return true;
  }
}

void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  geometry_msgs::Twist cmd_vel;
  double angle = msg->angle_min;
  double distance = msg->ranges[0];
  for (int i = 0; i < msg->ranges.size(); i++) {
    if (distance > msg->ranges[i]) {
      distance = msg->ranges[i];
      angle = msg->angle_min + i * msg->angle_increment;
    }
  }

  xpos = distance * cos(angle);
  ypos = distance * sin(angle);

  float p_gain_v = 0.1;
  float p_gain_a = 0.4;

  cmd_vel.linear.x = xpos > 0.4 ? xpos * p_gain_v : 0;
  cmd_vel.angular.z = xpos > 0.4 ? (ypos * p_gain_a) : 0;

  cmdvelpublisher_.publish(cmd_vel);
  ROS_INFO_STREAM_THROTTLE(2.0,"X position of pillar (m) : " << xpos);
  ROS_INFO_STREAM_THROTTLE(2.0,"Y position of pillar (m) : " << ypos);
  ROS_INFO_STREAM_THROTTLE(2.0,"Angle of pillar (rad) : " << angle);

  ROS_INFO_STREAM_THROTTLE(2.0,"Linear velocity (m/s) : " << cmd_vel.linear.x);
  ROS_INFO_STREAM_THROTTLE(2.0,"Angular velocity (rad/s) : " << cmd_vel.angular.z);

  ROS_INFO_STREAM_THROTTLE(2.0,"Successfully published.");

  pillarVisualization();
}

void SmbHighlevelController::pillarVisualization()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "rslidar";
  marker.header.stamp = ros::Time::now();
  marker.ns = "pillar_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = xpos;
  marker.pose.position.y = ypos;
  marker.pose.position.z = 0.1;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  markerpublisher_.publish(marker);
}
} /* namespace */