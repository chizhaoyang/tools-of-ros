#include <pointcloud_map_builder/tfToNavmsgs.h>

TfToNavmsgs::TfToNavmsgs(ros::NodeHandle& nh):_nh(nh){
  // get parameters
  if(!nh.getParam("/frame/source", _sourceFrame)) ROS_ERROR("Failed to get param 'frame/source'");
  if(!nh.getParam("/frame/target", _targetFrame)) ROS_ERROR("Failed to get param 'frame/target'");

  if(!nh.getParam("/topic/odometryTopic", _odometryTopic)) ROS_ERROR("Failed to get param 'topic/odometryTopic'");

  _pubOdom = nh.advertise<nav_msgs::Odometry>(_odometryTopic, 10);
}

void TfToNavmsgs::listenTFTrans(){
  // listen transform matrix from tf, M^T_C
  tf::StampedTransform truthOdomTrans;
  tf::TransformListener tfListener;
  try{
    ros::Time t = ros::Time(0);

    tfListener.waitForTransform(_targetFrame,  _sourceFrame, t, ros::Duration(1.0));

    tfListener.lookupTransform(_targetFrame,  _sourceFrame, t, truthOdomTrans);
  }
  catch (tf::TransformException &ex){
    ROS_ERROR("%s",ex.what());
  }
  nav_msgs::Odometry odom;
  geometry_msgs::Quaternion geoQuat;
  tf::quaternionTFToMsg(truthOdomTrans.getRotation(), geoQuat);

  odom.header.stamp = truthOdomTrans.stamp_;
  odom.header.frame_id = truthOdomTrans.frame_id_;
  odom.child_frame_id = truthOdomTrans.child_frame_id_;
  odom.pose.pose.orientation = geoQuat;
  odom.pose.pose.position.x = truthOdomTrans.getOrigin().x();
  odom.pose.pose.position.y = truthOdomTrans.getOrigin().y();
  odom.pose.pose.position.z = truthOdomTrans.getOrigin().z();

  _pubOdom.publish(odom);
}
