#pragma once

#include "husky_sensor_angle/Algorithm.hpp"


// ROS
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>

namespace ros_sensor_angle {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RosAngleCalculator
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  RosAngleCalculator(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RosAngleCalculator();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void topicCallback(const nav_msgs::Path& message);

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool serviceCallback(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;

  //! ROS topic name to subscribe to.
  std::string subscriberTopic_;

  //! ROS service server.
  ros::ServiceServer serviceServer_;

  //! Algorithm computation object.
  Algorithm algorithm_;
};

} /* namespace */
