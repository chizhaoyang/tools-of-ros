#include <pointcloud_map_builder/pointcloudMapBuilder.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

PointcloudMapBuilder::PointcloudMapBuilder(ros::NodeHandle& nh):_nh(nh){
  // get parameters
  if(!nh.getParam("/topic/pointcloud", _pointcloudTopic)) ROS_ERROR("Failed to get param 'topic/pointcloud'");
  if(!nh.getParam("/topic/globalMap", _globalMapTopic)) ROS_ERROR("Failed to get param 'topic/globalMap'");
  if(!nh.getParam("/frame/target", _targetFrame)) ROS_ERROR("Failed to get param 'frame/target'");
  if(!nh.getParam("/topic/odometryTopic", _odometryTopic)) ROS_ERROR("Failed to get param 'topic/odometryTopic'");

  if(!nh.getParam("/fileDirectory", _fileDirectory)) ROS_ERROR("Failed to get param '/lego_loam/fileDirectory'");

  _pubGlobalMap = nh.advertise<sensor_msgs::PointCloud2>(_globalMapTopic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicy;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* subCloud;
  message_filters::Subscriber<nav_msgs::Odometry>* subOdomTrans;
  message_filters::Synchronizer<SyncPolicy>* sync;
  subCloud = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, _pointcloudTopic, 10);
  subOdomTrans = new message_filters::Subscriber<nav_msgs::Odometry>(nh, _odometryTopic, 10);
  sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *subCloud, *subOdomTrans);
  sync->registerCallback(boost::bind(&PointcloudMapBuilder::pointcloudHandler, this, _1, _2));

  _downSizeFilter.setLeafSize(0.1, 0.1, 0.1);

  _cloudIn.reset(new pcl::PointCloud<PointType>());
  _globalMap.reset(new pcl::PointCloud<PointType>());
  _globalMapDS.reset(new pcl::PointCloud<PointType>());
}

void PointcloudMapBuilder::pointcloudHandler(const sensor_msgs::PointCloud2ConstPtr& pointcloudMsg, const nav_msgs::Odometry::ConstPtr& odomTrans){
  _cloudHeader = pointcloudMsg->header;

  pcl::fromROSMsg(*pointcloudMsg, *_cloudIn);

  // Remove Nan points.
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_cloudIn, *_cloudIn, indices);

  // get odometry transform.
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomTrans->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  _transfromCloudToMap[0] = roll;
  _transfromCloudToMap[1] = pitch;
  _transfromCloudToMap[2] = yaw;
  _transfromCloudToMap[3] = odomTrans->pose.pose.position.x;
  _transfromCloudToMap[4] = odomTrans->pose.pose.position.y;
  _transfromCloudToMap[5] = odomTrans->pose.pose.position.z;

  transfromCloudToMap();
}

void PointcloudMapBuilder::transfromCloudToMap(){
  float x1, y1, z1, x2, y2, z2;
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  size_t cloudSize;
  cloudSize = _cloudIn->points.size();
  cloudOut->resize(cloudSize);

  for (size_t i=0; i<cloudSize; ++i){
    // rotate with z axis (yaw).
    x1 = cos(-_transfromCloudToMap[2])*_cloudIn->points[i].x + sin(-_transfromCloudToMap[2]) *_cloudIn->points[i].y;
    y1 = -sin(-_transfromCloudToMap[2])*_cloudIn->points[i].x + cos(-_transfromCloudToMap[2]) *_cloudIn->points[i].y;
    z1 = _cloudIn->points[i].z;

    // rotate with y axis (pitch).
    z2 = cos(-_transfromCloudToMap[1])*z1 + sin(-_transfromCloudToMap[1])*x1;
    y2 = y1;
    x2 = -sin(-_transfromCloudToMap[1])*z1 + cos(-_transfromCloudToMap[1])*x1;

    // rotate with x axis (roll).
    cloudOut->points[i].x = x2 + _transfromCloudToMap[3];
    cloudOut->points[i].y = cos(-_transfromCloudToMap[0])*y2 + sin(-_transfromCloudToMap[0])*z2 + _transfromCloudToMap[4];
    cloudOut->points[i].z = -sin(-_transfromCloudToMap[0])*y2 + cos(-_transfromCloudToMap[0])*z2 + _transfromCloudToMap[5];
  }

  *_globalMap += *cloudOut;
  // std::cout<<"test"<<std::endl;
  // down sample global map to reduce points.
  _downSizeFilter.setInputCloud(_globalMap);
  _downSizeFilter.filter(*_globalMapDS);

  // save map.
  // save final point cloud
  pcl::io::savePCDFileASCII(_fileDirectory+"finalmap.pcd", *_globalMap);
  pcl::io::savePCDFileASCII(_fileDirectory+"finalmapDS.pcd", *_globalMapDS);  // for converting to grid map
  publishMap();
}

void PointcloudMapBuilder::publishMap(){
  sensor_msgs::PointCloud2 cloudMsgTemp;

  pcl::toROSMsg(*_globalMap, cloudMsgTemp);
  cloudMsgTemp.header.stamp = _cloudHeader.stamp;
  cloudMsgTemp.header.frame_id = _targetFrame;

  _pubGlobalMap.publish(cloudMsgTemp);
}
