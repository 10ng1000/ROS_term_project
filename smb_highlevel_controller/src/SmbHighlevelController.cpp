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
  if (!nodeHandle_.getParam("subscriber_topic", scanTopic_) || !nodeHandle_.getParam("queue_size", queueSize_)
      || !nodeHandle_.getParam("avoid_distance", avoidDistance_) || !nodeHandle_.getParam("speed", speed_)
      || !nodeHandle_.getParam("angular_speed", angularSpeed_)) return false;
  return true;
  }
}

void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  static bool arrive = 0;
  float disMinLeft, disMinRight;
  float angleMinLeft, angleMinRight;
  float distance;
  float angle;
  disMinRight = INF;
  angleMinRight = msg->angle_min;
  for (int i = 0; i < msg->ranges.size()/2; i++) {
    if (msg->ranges[i] < disMinRight) {
      disMinRight = msg->ranges[i];
      angleMinRight = msg->angle_min + i*msg->angle_increment;
    }
  }
  disMinLeft = msg->ranges[msg->ranges.size()/2];
  angleMinLeft = msg->angle_min;
  for (int i = msg->ranges.size()/2; i < msg->ranges.size(); i++) {
    if (msg->ranges[i] < disMinLeft) {
      disMinLeft = msg->ranges[i];
      angleMinLeft = msg->angle_min + i * msg->angle_increment;
    }
  }
  //ROS_INFO_STREAM_THROTTLE(1, "angleMinLeft: " << angleMinLeft << " disMinLeft: " << disMinLeft);
  //ROS_INFO_STREAM_THROTTLE(1, "angleMinRight: " << angleMinRight << " disMinRight: " << disMinRight);
  if (disMinLeft < disMinRight) {
    distance = disMinLeft;
    angle = angleMinLeft;
  }
  else {
    distance = disMinRight;
    angle = angleMinRight;
  }
  float xpos = distance * cos(angle);
  float ypos = distance * sin(angle);
  obstacleAvoidance(distance, disMinLeft, disMinRight, angleMinLeft, angleMinRight);
  //startCircularMotion(angle, angularSpeed_, distance);
  if(xpos > avoidDistance_ && arrive == 0) {
    findPillar(xpos, ypos, angle);
  }
  else {
    arrive = 1;
  }
}

void SmbHighlevelController::findPillar(float xpos, float ypos, float angle)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = xpos > 0.4 ? xpos * speed_ : 0;
  cmd_vel.angular.z = xpos > 0.4 ? (ypos * angularSpeed_) : 0;
  cmdvelpublisher_.publish(cmd_vel);

  ROS_INFO_STREAM_THROTTLE(2.0,"------Tracking pillar------");
  ROS_INFO_STREAM_THROTTLE(2.0,"Linear velocity (m/s) : " << cmd_vel.linear.x);
  ROS_INFO_STREAM_THROTTLE(2.0,"Angular velocity (rad/s) : " << cmd_vel.angular.z);
}

void SmbHighlevelController::startCircularMotion(float startAngle, float angular, float radius)
{
  static bool ini = false;
  static float fixedRadius = 0;
  if (!ini) iniCircularMotion(startAngle, radius, ini, fixedRadius);
  //else circularMotion(angular, fixedRadius);
}

void SmbHighlevelController::circularMotion(float angular, float radius)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = angular * radius; //This does not work, so don't execute this function
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = - angular;
  cmdvelpublisher_.publish(cmd_vel);
  ROS_INFO_STREAM_THROTTLE(2.0,"------Moving------");
  ROS_INFO_STREAM_THROTTLE(2.0,"Linear velocity (m/s) : " << cmd_vel.linear.x);
  ROS_INFO_STREAM_THROTTLE(2.0,"Angular velocity (rad/s) : " << cmd_vel.angular.z);
}

void SmbHighlevelController::iniCircularMotion(float angle, float distance, bool &ini, float &fixedRadius)
{
  geometry_msgs::Twist cmd_vel;
  if (angle > 1.56 && angle < 1.6 || angle > -1.6 && angle < -1.56) {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    fixedRadius = distance;
    ini = true;
    cmdvelpublisher_.publish(cmd_vel);
    ROS_INFO_STREAM_THROTTLE(2.0,"Robot is in the correct position.");
    return;
  }
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z =  angularSpeed_ * (1.7 - angle) * 0.8;
  ini = false;
  cmdvelpublisher_.publish(cmd_vel);
  ROS_INFO_STREAM_THROTTLE(2.0, "Waiting for the robot to be in the correct position.");
}

void SmbHighlevelController::obstacleAvoidance(float distance, float disMinLeft, float disMinRight, float angleMinLeft, float angleMinRight)
{
  static bool avoiding = false;
  if (distance < avoidDistance_) {
    avoiding = true;
  }
  if (avoiding == false) return;
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = speed_;
  if ((disMinLeft >= 2 * avoidDistance_ + 1 && disMinRight >= 2 * avoidDistance_ + 1) && avoiding == true) {
     avoiding = false;
     cmd_vel.angular.z = 0;
     cmdvelpublisher_.publish(cmd_vel);
     ROS_INFO_STREAM_THROTTLE(2.0, "------Successfully Aoivded.------"); 
     return;
  }
  cmd_vel.linear.x = (distance - avoidDistance_) * speed_;
  if (disMinLeft >= disMinRight) cmd_vel.angular.z = disMinLeft > 3 * avoidDistance_? 60 * avoidDistance_ * angularSpeed_ :disMinLeft * angularSpeed_ * 20;
  else cmd_vel.angular.z = - (disMinRight > 3 * avoidDistance_? 60 * avoidDistance_ * angularSpeed_ :disMinRight * angularSpeed_ * 20);
  cmdvelpublisher_.publish(cmd_vel);
  ROS_INFO_STREAM_THROTTLE(2.0,"------Obstacle Avoiding------");
  ROS_INFO_STREAM_THROTTLE(2.0,"Linear velocity (m/s) : " << cmd_vel.linear.x);
  ROS_INFO_STREAM_THROTTLE(2.0,"Angular velocity (rad/s) : " << cmd_vel.angular.z);
}

} /* namespace */