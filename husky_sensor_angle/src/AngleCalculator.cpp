#include "husky_sensor_angle/AngleCalculator.hpp"

// STD
#include <string>

namespace ros_sensor_angle {

RosAngleCalculator::RosAngleCalculator(ros::NodeHandle& nodeHandle)
 : nodeHandle_(nodeHandle), tfListener_(tfBuffer_)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &RosAngleCalculator::topicCallback, this);
  serviceServer_ = nodeHandle_.advertiseService("get_average",
                                                &RosAngleCalculator::serviceCallback, this);

  angle_pub_ = nodeHandle_.advertise<std_msgs::Float64>("/husky_joint_controller/command", 1, true);

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
  geometry_msgs::PoseStamped pose_in;
  geometry_msgs::PoseStamped pose_out;
  double x, y, angle;
  pose_in = message.poses.back();
  try
  {
    tfBuffer_.transform(pose_in,pose_out,"camera_mount",ros::Duration(3.0));
    // ROS_INFO("point of target in frame of camera Position(x:%f y:%f z:%f)\n", 
      pose_out.pose.position.x,
      pose_out.pose.position.y,
      pose_out.pose.position.z);
    x = pose_out.pose.position.x;
    y = pose_out.pose.position.y;

    if(sqrt(x*x+y*y) > 1){
      angle = atan2(y,x);
      ROS_INFO_STREAM("Target angle:"<<angle);
    }
    else{
      angle = 0;
      ROS_INFO_STREAM("Target angle:"<<angle);
    }

    std_msgs::Float64 msg;

    msg.data = angle;
    angle_pub_.publish(msg);

  }
  catch (tf2::TransformException &ex) 
  {
    ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
  }

  //ROS_INFO_STREAM(message.poses.back());
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
