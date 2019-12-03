// subscribe tf transformation and publish nav_msgs::Odometry

// Author: Chizhao Yang
#ifndef TFTONAVMSGS_H
#define TFTONAVMSGS_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>

class TfToNavmsgs{
public:
  TfToNavmsgs(ros::NodeHandle& nh);
  ~TfToNavmsgs() = default;

  void listenTFTrans();

private:

  ros::NodeHandle _nh;
  ros::Publisher _pubOdom;

  std::string _sourceFrame;
  std::string _targetFrame;

  std::string _odometryTopic;

  std_msgs::Header _header;
};

#endif
