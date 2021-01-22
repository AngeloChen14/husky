#include "husky_sensor_angle/AngleCalculator.hpp"

// STD
#include <string>

namespace ros_sensor_angle {

RosAngleCalculator::RosAngleCalculator(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &RosAngleCalculator::topicCallback, this);
  serviceServer_ = nodeHandle_.advertiseService("get_average",
                                                &RosAngleCalculator::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
}

RosAngleCalculator::~RosAngleCalculator()
{
}

bool RosAngleCalculator::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void RosAngleCalculator::topicCallback(const nav_msgs::Path& message)
{
  geometry_msgs::PoseStamped target_pose;
  target_pose = message.poses.back();
  
  ROS_INFO_STREAM(message.poses.back());
  // ROS_INFO_STREAM(message);
  // algorithm_.addData(message.poses.end);
}

bool RosAngleCalculator::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}

} /* namespace */
