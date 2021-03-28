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
  feedback_sub_ = nodeHandle_.subscribe(feedbackSubTopic_, 100,
                                    &RosAngleCalculator::feedbackCallback, this);
  // serviceServer_ = nodeHandle_.advertiseService("get_average",
  //                                               &RosAngleCalculator::serviceCallback, this);
  gazeboClient_ = nodeHandle_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  angle_pub_ = nodeHandle_.advertise<std_msgs::Float64>("/husky_joint_controller/command", 100, true);
  score_pub_ = nodeHandle_.advertise<std_msgs::Float64>("/angle_score", 100, true);
  nav_pub_   = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100, true);
  timer1_ = nodeHandle_.createTimer(ros::Duration(0.1),&RosAngleCalculator::timer1Callback,this); 
  // timer2_ = nodeHandle_.createTimer(ros::Duration(0.1),&RosAngleCalculator::timer2Callback,this);  

  ROS_INFO("Successfully launched node.");
}

RosAngleCalculator::~RosAngleCalculator()
{
}

bool RosAngleCalculator::readParameters()
{
  if (!nodeHandle_.getParam("pos_subscriber_topic", posSubTopic_)) return false;
  if (!nodeHandle_.getParam("scan_subscriber_topic", scanSubTopic_)) return false;
  if (!nodeHandle_.getParam("feedback_subscriber_topic", feedbackSubTopic_)) return false;
  return true;
}

void RosAngleCalculator::posCallback(const nav_msgs::Path& message)
{
  nav_msgs_ = message;
  // std_msgs::Float64 msg;
  // msg.data = calculateAngle(nav_msgs_, laser_msgs_);
  // angle_pub_.publish(msg);
}

void RosAngleCalculator::scanCallback(const sensor_msgs::LaserScan& message)
{
  laser_msgs_ = message;
  // std_msgs::Float64 msg;
  // if(nav_msgs_.header.seq!=0 && laser_msgs_.header.seq!=0)
  // {
  //   ros::Time begin = ros::Time::now();
  //   msg.data = calculateAngle(nav_msgs_, laser_msgs_);
  //   angle_pub_.publish(msg);
  //   ros::Time end = ros::Time::now();
  //   ROS_INFO_STREAM("Begin:"<<begin<<'\n'<<"End:"<<end);
  //   // msg.data  = calculateScore(nav_msgs_, laser_msgs_);
  //   // score_pub_.publish(msg);
  // }
}

void RosAngleCalculator::feedbackCallback(const sensor_msgs::JointState& message)
{
    fb_angle_=message.position.back();
}

void RosAngleCalculator::timer1Callback(const ros::TimerEvent& e)
{
  std_msgs::Float64 msg;
  static uint count = 0;
  static bool SensorReadyFlag = false;
  if(!SensorReadyFlag && getTargetPose() &&laser_msgs_.header.seq!=0)
  {
      SensorReadyFlag = true;
      setNavGoal();
  }
  // getTargetPose();

  if(SensorReadyFlag)
  {
    getTargetPose();
    msg.data = calculateAngle(nav_msgs_, laser_msgs_);
    angle_pub_.publish(msg);

    msg.data  = calculateScore(nav_msgs_, laser_msgs_);
    score_pub_.publish(msg);
    if(count>10)
    {    
      setNavGoal();
      count=0;
    }
    count++;
  }
}

// bool RosAngleCalculator::serviceCallback(std_srvs::Trigger::Request& request,
//                                          std_srvs::Trigger::Response& response)
// {
//   response.success = true;
//   response.message = "The average is " + std::to_string(algorithm_.getAverage());
//   return true;
// }

double RosAngleCalculator::calculateAngle(const nav_msgs::Path& nav_message, const sensor_msgs::LaserScan& laser_message) 
{
  double angle,target_angle,path_angle;
  // geometry_msgs::PoseStamped target_pose;

  // geometry_msgs::PoseStamped target_pose = nav_message.poses.back();

  target_angle = calculateAngle_Target(target_pose_);
  path_angle = calculateAngle_Path(laser_message);
  
  // x = target_pose.pose.position.x;
  // y = target_pose.pose.position.y;
  // if(sqrt(x*x+y*y)<1)
  //   angle  = 0;
  // else
  // {
  //Fixed Policy
  // target_angle=0;
  // angle = target_angle;
  
  //Visual Servo Policy
  angle = path_angle*0.5 + target_angle*0.5;
  // angle = (target_angle + path_angle)/2;

  // 3 Stage Policy
  // if(abs(target_angle)<fov_angle/2) 
  //   angle = target_angle;
  // else if (abs(target_angle) < fov_angle)
  // {
  //   angle = fov_angle/2 * copysign(1.0,target_angle);
  // }
  //   else
  // {
  //   angle = (abs(target_angle) - fov_angle/2) * copysign(1.0,target_angle);
  // }

  // Half Policy 
  // if (abs(target_angle) < fov_angle)
  // {
  //   angle = target_angle/2;
  // }
  // else
  // {
  //   angle = (abs(target_angle) - fov_angle/2) * copysign(1.0,target_angle);
  // }

  // Test Policy 
  // float n = 2;
  // if (abs(target_angle) < fov_angle*n/(2*(n-1)))
  // {
  //   angle = target_angle/n;
  // }
  // else
  // {
  //   angle = (abs(target_angle) - fov_angle/2) * copysign(1.0,target_angle);
  // }

  // }

  return angle;
}

double RosAngleCalculator::calculateAngle_Target(const geometry_msgs::PoseStamped target_message)
{
  geometry_msgs::PoseStamped pose_out;geometry_msgs::TransformStamped transformStamped;
  double x=0, y=0, target_angle=0;
  try
  {
    transformStamped = tfBuffer_.lookupTransform("camera_mount", target_message.header.frame_id, ros::Time(0),ros::Duration(0.5));
    tf2::doTransform(target_message,pose_out,transformStamped);
  }
  catch (tf2::TransformException &ex) 
  {
    ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
  }

  x = pose_out.pose.position.x;
  y = pose_out.pose.position.y;

  if(sqrt(x*x+y*y) > 1) target_angle = atan2(y,x);
  else target_angle = 0;

  return target_angle;
}

double RosAngleCalculator::calculateAngle_Path(const sensor_msgs::LaserScan& laser_message)
{
  double gain_factor = 1.5; // gain factor for inf range data
  double path_angle,range_sum = 0,index_sum=0;
  for(uint index=0;index<laser_message.ranges.size();++index)
  {
    if(isfinite(laser_message.ranges[index]))
    {
      index_sum += index*(laser_message.ranges[index]-laser_message.range_min);
      range_sum += (laser_message.ranges[index]-laser_message.range_min);  
    }
    else
    {
       index_sum += index*(laser_message.range_max-laser_message.range_min)*gain_factor;
       range_sum += (laser_message.range_max-laser_message.range_min)*gain_factor;
    }
  }
  // ROS_INFO_STREAM("Index sum is:"<<index_sum<<"Range sum is:"<<range_sum);
  path_angle = (double)index_sum/range_sum*laser_message.angle_increment+laser_message.angle_min + fb_angle_;
  // ROS_INFO_STREAM("Path angle is:"<<(path_angle-fb_angle_)/M_PI*180);
  return path_angle;
}

double RosAngleCalculator::calculateScore(const nav_msgs::Path& nav_message, const sensor_msgs::LaserScan& laser_message) //laserscan paramater(rad)
{
  double score, target_score, scan_score;
  

  target_score = calculateScore_Target(target_pose_,laser_message);
  // path_score = calculateScore_Path(nav_message,laser_message);
  scan_score = calculateScore_Scan(laser_message,0.05);

  score = target_score + scan_score;


  // geometry_msgs::PoseStamped pose_out;
  // start = nav_message.poses.size()>200 ? nav_message.poses.size()-200 : 0; // remove unnessary nav goals
  // geometry_msgs::TransformStamped transformStamped;

  // try
  // {
  //   transformStamped = tfBuffer_.lookupTransform("camera_link", nav_message.poses.back().header.frame_id, ros::Time(0),ros::Duration(0.5));
  // }
  // catch (tf2::TransformException &ex) 
  // {
  //   ROS_WARN("Transform Failure %s\n", ex.what()); //Print exception which was caught
  // }
  
  // for(uint i=start;i<nav_message.poses.size();++i){
  //   try
  //   {
  //     tf2::doTransform(nav_message.poses[i],pose_out,transformStamped);
  //   }
  //   catch (tf2::TransformException &ex) 
  //   {
  //     ROS_WARN("Transform Failure %s\n", ex.what()); //Print exception which was caught
  //     break;
  //   }
  //   x = pose_out.pose.position.x;
  //   y = pose_out.pose.position.y;
  //   angle = atan2(y,x);
  //   if(x>dep_min && x<dep_max && abs(angle) < fov_angle/2) //consider whether path point is in 
  //   {
  //     float point_distance,scan_distance; uint index;
  //     index = round((angle - laser_message.angle_min)/laser_message.angle_increment);
  //     if(index < 0) //prevent out_of_range error
  //       index = 0;
  //     else if(index >= laser_message.ranges.size()) 
  //       index = laser_message.ranges.size()-1;
  //     scan_distance = laser_message.ranges[index];
  //     if (!isnan(scan_distance))
  //     {
  //       point_distance = sqrt(x*x+y*y);
  //       if(point_distance < scan_distance) count+=1;
  //     }
  //     else
  //       count +=1;
  //   } 
  //   if(x>dep_min) len++;  //only count path > fov_min
  // }

  // if(sqrt(x*x+y*y) > dep_min)
  // {
  //   if(len==0) len+=1;
  //   path_score = (double)count/len;
    
  //   // if(x>fov_min && x<fov_max && abs(angle) < fov_angle/2) target_score = 1;  //extra score for successful target tracking
  //   // else target_score = 0; 
  //   target_score =  - angle*angle/(fov_angle*fov_angle)  + 1; // score for successful target tracking, 
  //   target_score = (target_score > 0.75) ? target_score : 0;
  //   if( target_score < 0.75) 
  //   {
  //     target_score = 0;
  //   }
  //   score = target_score + path_score;
  // }

  // else
  //   score = 2;
  // ROS_INFO_STREAM("Score is:"<<score);
  return score;
}

double RosAngleCalculator::calculateScore_Target(const geometry_msgs::PoseStamped target_message, const sensor_msgs::LaserScan& laser_message)
{
  double x,y,angle,dep_min,dep_max,fov_angle,score;
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseStamped pose_out;
  try
  {
    transformStamped = tfBuffer_.lookupTransform("camera_link", target_message.header.frame_id, ros::Time(0),ros::Duration(0.5));
     tf2::doTransform(target_message,pose_out,transformStamped);
  }
  catch (tf2::TransformException &ex) 
  {
    ROS_WARN("Transform Failure %s\n", ex.what()); //Print exception which was caught
  }

  x = pose_out.pose.position.x;
  y = pose_out.pose.position.y;
  angle = atan2(y,x);
  dep_min = laser_message.range_min;
  dep_max = laser_message.range_max;
  fov_angle = laser_message.angle_max - laser_message.angle_min;

  if(x > dep_min && x < dep_max)
  {
    // if(x>fov_min && x<fov_max && abs(angle) < fov_angle/2) target_score = 1;  //extra score for successful target tracking
    // else target_score = 0; 
    score =  - 2*angle*angle/(fov_angle*fov_angle)  + 1;  
    score = (score > 0.5) ? score : 0;
  }
  else if(x < dep_min)
    score = 1;
  else
    score = 0;
  // ROS_INFO_STREAM("Target Score is:"<<score);
  return score;
}

double RosAngleCalculator::calculateScore_Path(const nav_msgs::Path& nav_message, const sensor_msgs::LaserScan& laser_message)
{
  int len=0, count=0,start=0; double x, y,angle,dep_max,dep_min,fov_angle,path_score;
  if(nav_msgs_.header.seq==0) return 1;
  geometry_msgs::PoseStamped pose_out;
  start = nav_message.poses.size()>200 ? nav_message.poses.size()-200 : 0; // remove unnessary nav goals
  geometry_msgs::TransformStamped transformStamped;
  dep_min = laser_message.range_min;
  dep_max = laser_message.range_max;
  fov_angle = laser_message.angle_max - laser_message.angle_min;

  try
  {
    transformStamped = tfBuffer_.lookupTransform("camera_link", nav_message.poses.back().header.frame_id, ros::Time(0),ros::Duration(0.5));
  }
  catch (tf2::TransformException &ex) 
  {
    ROS_WARN("Transform Failure %s\n", ex.what()); //Print exception which was caught
  }
  
  for(uint i=start;i<nav_message.poses.size();++i){
    tf2::doTransform(nav_message.poses[i],pose_out,transformStamped);
    x = pose_out.pose.position.x;
    y = pose_out.pose.position.y;
    angle = atan2(y,x);
    if(x>dep_min && x<dep_max && abs(angle) < fov_angle/2) //consider whether path point is in fov
    {
      float scan_distance; uint index;
      index = round((angle - laser_message.angle_min)/laser_message.angle_increment);
      if(index < 0) //prevent out_of_range error
        index = 0;
      else if(index >= laser_message.ranges.size()) 
        index = laser_message.ranges.size()-1;
      scan_distance = laser_message.ranges[index];

      if (isfinite(scan_distance))
      {
        if(sqrt(x*x+y*y) < scan_distance) count+=1;
      }
      else
        count +=1;
    } 
    if(x>dep_min) len++;  //only count path > fov_min
  }

  if(len>0)
    path_score = (double)count/len;
  else
    path_score = 1;

  return path_score;
}

double RosAngleCalculator::calculateScore_Scan(const sensor_msgs::LaserScan& laser_message,float weight_factor)
{
  int obstacle_threshold = 5; //num of continuous finite ranges before counting as obstacle
  int road_threshold = 5; //num of continuous infinite ranges before counting as travelable area
  double fov_full,fov_sum=0,scan_score;
  bool obstacle_flag = false; // true for obstacle, flase for travelable area
  int obstacle_num=0,obstacle_count=0,road_count=0;
  fov_full = (laser_message.angle_max-laser_message.angle_min)*(laser_message.range_max-laser_message.range_min);

  for(auto range:laser_message.ranges)
  {
    if(isfinite(range)){
      fov_sum += laser_message.angle_increment*(range-laser_message.range_min);
      if(!obstacle_flag){
        obstacle_count++;
        if(obstacle_count>=obstacle_threshold){
          obstacle_flag = true;
          obstacle_num++;
        }
      }
      else{
        obstacle_count = 0;
        road_count = 0;
      }      
    }
    else{
      fov_sum += laser_message.angle_increment*(laser_message.range_max-laser_message.range_min);
      if(obstacle_flag){
        road_count++;
        if(road_count>=road_threshold){
          obstacle_flag = false;
        }
      }
      else{
        obstacle_count = 0;
        road_count = 0;
      }
    }      
  }
  // ROS_INFO_STREAM("Num is:"<<obstacle_num<<" Percentage is:"<<fov_sum/fov_full);
  scan_score = fov_sum/fov_full + weight_factor*obstacle_num;
  return scan_score;
}

bool RosAngleCalculator::getTargetPose()
{
  double x,y,angle,dep_min,dep_max,fov_angle;
  geometry_msgs::PoseStamped feedbackpos,pose_out;
  gazebo_msgs::GetModelState model_srv;
  model_srv.request.model_name = "target";
  model_srv.request.relative_entity_name = '/';
  if(gazeboClient_.call(model_srv))
  {
    feedbackpos.header = model_srv.response.header;
    feedbackpos.header.frame_id = "base_link";
    feedbackpos.pose.position = model_srv.response.pose.position;
    feedbackpos.pose.position.z = 0;
    feedbackpos.pose.orientation.w = 1.0;
    try{
      tfBuffer_.transform(feedbackpos,pose_out,"camera_link",ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
      return false;
    }
    dep_min = laser_msgs_.range_min;
    dep_max = laser_msgs_.range_max;
    fov_angle = laser_msgs_.angle_max - laser_msgs_.angle_min;
    x = pose_out.pose.position.x;
    y = pose_out.pose.position.y;
    angle = atan2(y,x);
    if(x>dep_min && x<dep_max && abs(angle) < fov_angle/2) //consider whether target point is in fov
    {
      target_pose_ = feedbackpos;
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

void RosAngleCalculator::setNavGoal()
{
  geometry_msgs::PoseStamped pose_out;
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer_.lookupTransform("map", target_pose_.header.frame_id, ros::Time(0),ros::Duration(0.1));
    tf2::doTransform(target_pose_,pose_out,transformStamped);
    pose_out.pose.position.z = 0;

  }
  catch (tf2::TransformException &ex) 
  {
    ROS_WARN("Transform Failure %s\n", ex.what()); //Print exception which was caught
  }
  nav_pub_.publish(pose_out);
}

} /* namespace */