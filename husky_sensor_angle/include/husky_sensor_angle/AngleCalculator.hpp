#pragma once

#include "husky_sensor_angle/Algorithm.hpp"


// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/Float64.h"

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
  void posCallback(const nav_msgs::Path& message);
  void scanCallback(const sensor_msgs::LaserScan& message);
  void timer1Callback(const ros::TimerEvent& e);

  double calculateAngle(const nav_msgs::Path& nav_message, const sensor_msgs::LaserScan& laser_message, float fov_angle = 50.0*M_PI/180);
  double calculateScore(const nav_msgs::Path& nav_message, const sensor_msgs::LaserScan& laser_message, float fov_min=0.4, float fov_max=5.0, float fov_angle=60.0*M_PI/180, // fov parameter(m,m,rad)
                                                                                                  float laser_min = -0.52, float laser_incre = 0.005);



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
  ros::Subscriber pos_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher angle_pub_;
  ros::Publisher score_pub_;
  ros::Timer timer1_;

  //! ROS topic name to subscribe to.
  std::string posSubTopic_;
  std::string scanSubTopic_;

  //! ROS service server.
  ros::ServiceServer serviceServer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  //Algorithm computation object.  
  Algorithm algorithm_; 
  nav_msgs::Path nav_msgs_;
  sensor_msgs::LaserScan laser_msgs_;
};
} /* namespace */
