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
  pos_sub_ = nodeHandle_.subscribe(posSubTopic_, 100,
                                      &RosAngleCalculator::posCallback, this);
  scan_sub_ = nodeHandle_.subscribe(scanSubTopic_, 1000,
                                      &RosAngleCalculator::scanCallback, this);

  // serviceServer_ = nodeHandle_.advertiseService("get_average",
  //                                               &RosAngleCalculator::serviceCallback, this);

  angle_pub_ = nodeHandle_.advertise<std_msgs::Float64>("/husky_joint_controller/command", 100, true);
  score_pub_ = nodeHandle_.advertise<std_msgs::Float64>("/angle_score", 100, true);
  timer1_ = nodeHandle_.createTimer(ros::Duration(0.02),&RosAngleCalculator::timer1Callback,this);  

  ROS_INFO("Successfully launched node.");
}

RosAngleCalculator::~RosAngleCalculator()
{
}

bool RosAngleCalculator::readParameters()
{
  if (!nodeHandle_.getParam("pos_subscriber_topic", posSubTopic_)) return false;
  if (!nodeHandle_.getParam("scan_subscriber_topic", scanSubTopic_)) return false;
  return true;
}

void RosAngleCalculator::posCallback(const nav_msgs::Path& message)
{
  geometry_msgs::PoseStamped pose_in;
  geometry_msgs::PoseStamped pose_out;
  std_msgs::Float64 msg;
  double x=0, y=0, target_angle=0, angle;
  double fov_angle=50.0*M_PI/180;

  pose_in = message.poses.back();
  try
  {
    tfBuffer_.transform(pose_in,pose_out,"camera_mount",ros::Duration(3.0));
    // ROS_INFO("point of target in frame of camera Position(x:%f y:%f z:%f)\n", 
      // pose_out.pose.position.x,
      // pose_out.pose.position.y,
      // pose_out.pose.position.z);
    x = pose_out.pose.position.x;
    y = pose_out.pose.position.y;
    if(sqrt(x*x+y*y) > 1) target_angle = atan2(y,x);
  }
  catch (tf2::TransformException &ex) 
  {
    ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
  }

  if(abs(target_angle)<fov_angle/2) 
    angle = target_angle;
  else if (abs(target_angle) < fov_angle)
  {
    angle = fov_angle/2 * copysign(1.0,target_angle);
  }
  else
  {
    angle = (abs(target_angle) - fov_angle/2) * copysign(1.0,target_angle);
  }
  msg.data = angle;
  angle_pub_.publish(msg);
  nav_msgs_ = message;

  //ROS_INFO_STREAM(message.poses.back());
  // ROS_INFO_STREAM(message);
  // algorithm_.addData(message.poses.end);
}


void RosAngleCalculator::scanCallback(const sensor_msgs::LaserScan& message)
{
   laser_msgs_ = message;
}

bool RosAngleCalculator::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}

double RosAngleCalculator::calculateScore(const nav_msgs::Path& nav_message, const sensor_msgs::LaserScan& laser_message,
                                              float fov_min=0.4, float fov_max=5.0, float fov_angle=60.0*M_PI/180, // fov parameter(m,m,rad)
                                              float laser_min = -0.52, float laser_max = 0.52, float laser_incre = 0.005 ) //laserscan paramater(rad)
{
  int len=0, count=0,start=0; double x, y, angle, score, target_score, path_score;
  geometry_msgs::PoseStamped pose_out;
  start = nav_message.poses.size()>200 ? nav_message.poses.size()-200 : 0; // remove unnessary nav goals
  // ROS_INFO_STREAM("Start pos is:"<<start);
  for(uint i=start;i<nav_message.poses.size();i++){
    try
    {
      tfBuffer_.transform(nav_message.poses[i],pose_out,"camera_link",ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Transform Failure %s\n", ex.what()); //Print exception which was caught
      break;
    }
    x = pose_out.pose.position.x;
    y = pose_out.pose.position.y;
    angle = atan2(y,x);
    if(x>fov_min && x<fov_max && abs(angle) < fov_angle/2) //consider whether path point is in 
    {
      float point_distance,scan_distance; uint index;
      index = round((angle - laser_min)/laser_incre);
      if(index < 0) //prevent out_of_range error
        index = 0;
      else if(index >= laser_message.ranges.size()) 
        index = laser_message.ranges.size()-1;
      scan_distance = laser_message.ranges[index];
      if (!isinf(scan_distance))
      {
        point_distance = sqrt(x*x+y*y);
        if(point_distance < scan_distance) count+=1;
      }
      else
        count +=1;
    } 
    if(x>fov_min) len++;  //only count path > fov_min
  }

  if(sqrt(x*x+y*y)>fov_min)
  {
    if(len==0) len+=1;
    path_score = (double)count/len;
    // if(x>fov_min && x<fov_max && abs(angle) < fov_angle/2) score += 1;  //extra score for successful target tracking
    target_score =  - 2*angle*angle/(fov_angle*fov_angle)  + 1; // score for successful target tracking, 
    // target_score = (target_score > 0.5) ? target_score : 0;
    if( target_score < 0.5) 
    {
      target_score = 0;
    }
    score = target_score + path_score;
  }

  else
    score = 2;
  // ROS_INFO_STREAM("Score is:"<<score);
  return score;
}

void RosAngleCalculator::timer1Callback(const ros::TimerEvent& e)
{
  double score;std_msgs::Float64 msg;
  score= calculateScore(nav_msgs_, laser_msgs_);
  msg.data = score;
  score_pub_.publish(msg);
}

} /* namespace */