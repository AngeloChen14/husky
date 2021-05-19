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
#include "std_msgs/Float64MultiArray.h"
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/GetModelState.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <random>

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
  void posCallback(const visualization_msgs::MarkerArray& message);
  void scanCallback(const sensor_msgs::LaserScan& message);
  void feedbackCallback(const sensor_msgs::JointState& message);
  void timer1Callback(const ros::TimerEvent& e);

  double calculateAngle(const visualization_msgs::MarkerArray& body_message, const sensor_msgs::LaserScan& laser_message);
  double calculateAngle_Target(const geometry_msgs::PoseStamped);
  double calculateAngle_Path(const sensor_msgs::LaserScan& laser_message);

  double calculateScore(const nav_msgs::Path& nav_message, const sensor_msgs::LaserScan& laser_message);
  double calculateScore_Target(const geometry_msgs::PoseStamped, const sensor_msgs::LaserScan& laser_message);
  double calculateScore_Path(const nav_msgs::Path& nav_message, const sensor_msgs::LaserScan& laser_message);  
  double calculateScore_Scan(const sensor_msgs::LaserScan& laser_message,float weight_factor);  
  bool getTargetPose();
  void setNavGoal(const geometry_msgs::PoseStamped& target_message);
  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  // bool serviceCallback(std_srvs::Trigger::Request& request,
  //                      std_srvs::Trigger::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber pos_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber feedback_sub_;
  ros::Publisher angle_pub_;
  ros::Publisher score_pub_;
  ros::Publisher nav_pub_;
  ros::Publisher target_pub_;
  ros::Timer timer1_;

  //! ROS topic name to subscribe to.
  std::string posSubTopic_;
  std::string scanSubTopic_;
  std::string feedbackSubTopic_;
  std::string globalFrame_;

  //! ROS service server.
  // ros::ServiceServer serviceServer_;
  // ros::ServiceClient gazeboClient_;
  ros::ServiceClient gazeboClient_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  //Algorithm computation object.  
  Algorithm algorithm_; 
  nav_msgs::Path nav_msgs_;
  sensor_msgs::LaserScan laser_msgs_;
  visualization_msgs::MarkerArray body_msgs_;
  geometry_msgs::PoseStamped target_pose_;      // noisy data for futher process
  // geometry_msgs::PoseStamped target_pose_true_; // noiseless data for evalutaion
  double fb_angle_;
  bool target_found_flag_;
};
} /* namespace */
