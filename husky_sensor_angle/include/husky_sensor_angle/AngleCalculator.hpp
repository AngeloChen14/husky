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
#include <sensor_msgs/JointState.h>

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
  void feedbackCallback(const sensor_msgs::JointState& message);
  void timer1Callback(const ros::TimerEvent& e);

  double calculateAngle(const nav_msgs::Path& nav_message, const sensor_msgs::LaserScan& laser_message);
  double calculateAngle_Target(const geometry_msgs::PoseStamped);
  double calculateAngle_Path(const sensor_msgs::LaserScan& laser_message);

  double calculateScore(const nav_msgs::Path& nav_message, const sensor_msgs::LaserScan& laser_message);
  double calculateScore_Taget(const geometry_msgs::PoseStamped, const sensor_msgs::LaserScan& laser_message);
  double calculateScore_Path(const nav_msgs::Path& nav_message, const sensor_msgs::LaserScan& laser_message);  
  double calculateScore_Scan(const sensor_msgs::LaserScan& laser_message,float weight_factor);  
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
  ros::Subscriber feedback_sub_;
  ros::Publisher angle_pub_;
  ros::Publisher score_pub_;
  ros::Timer timer1_;

  //! ROS topic name to subscribe to.
  std::string posSubTopic_;
  std::string scanSubTopic_;
  std::string feedbackSubTopic_;

  //! ROS service server.
  ros::ServiceServer serviceServer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  //Algorithm computation object.  
  Algorithm algorithm_; 
  nav_msgs::Path nav_msgs_;
  sensor_msgs::LaserScan laser_msgs_;
  double fb_angle_;
};
} /* namespace */
