#include <pointcloud_map_builder/pointcloudMapBuilder.h>
//
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

PointcloudMapBuilder::PointcloudMapBuilder(ros::NodeHandle& nh):_nh(nh){
  // get parameters
  if(!nh.getParam("/topic/pointcloud", _pointcloudTopic)) ROS_ERROR("Failed to get param 'topic/pointcloud'");
  if(!nh.getParam("/topic/globalMap", _globalMapTopic)) ROS_ERROR("Failed to get param 'topic/globalMap'");
  // if(!nh.getParam("/frame/source", _sourceFrame)) ROS_ERROR("Failed to get param 'frame/source'");
  if(!nh.getParam("/frame/target", _targetFrame)) ROS_ERROR("Failed to get param 'frame/target'");

  _pubGlobalMap = nh.advertise<sensor_msgs::PointCloud2>(_globalMapTopic, 1);

  _subCloud = nh.subscribe<sensor_msgs::PointCloud2>(_pointcloudTopic, 1, &PointcloudMapBuilder::pointcloudHandler, this);

  _cloudIn.reset(new pcl::PointCloud<PointType>());
  _globalMap.reset(new pcl::PointCloud<PointType>());

}

void PointcloudMapBuilder::pointcloudHandler(const sensor_msgs::PointCloud2ConstPtr& pointcloudMsg){
  _cloudHeader = pointcloudMsg->header;

  pcl::fromROSMsg(*pointcloudMsg, *_cloudIn);

  // Remove Nan points.
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_cloudIn, *_cloudIn, indices);

  listenTFTrans();
}

void PointcloudMapBuilder::listenTFTrans(){

  // listen transform matrix from tf, M^T_C
  tf::StampedTransform truthOdomTrans;
  tf::TransformListener tfListener;
  try{
    ros::Time t = ros::Time(0);
    // tfListener.waitForTransform(_targetFrame,  _cloudHeader.frame_id, _cloudHeader.stamp, ros::Duration(1.0));
    tfListener.waitForTransform(_targetFrame,  _cloudHeader.frame_id, t, ros::Duration(1.0));

    // tfListener.lookupTransform(_targetFrame, _cloudHeader.frame_id, _cloudHeader.stamp, truthOdomTrans);
    // tfListener.lookupTransform(_targetFrame,  _cloudHeader.frame_id, _cloudHeader.stamp, truthOdomTrans);
    tfListener.lookupTransform(_targetFrame,  _cloudHeader.frame_id, t, truthOdomTrans);
  }
  catch (tf::TransformException &ex){
    ROS_ERROR("%s",ex.what());
    // ros::Duration(1.0).sleep();
    // return;
  }

  double roll, pitch, yaw;
  // geometry_msgs::Quaternion geoQuat
  // tf::quaternionTFToMsg(truthOdomTrans.getRotation(), geoQuat);
  // geometry_msgs::Quaternion geoQuat = truthOdomTrans.getRotation();
  // in this case, rotate seq: yaw pitch raw
  // tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
  tf::Matrix3x3(truthOdomTrans.getRotation()).getRPY(roll, pitch, yaw);

  _transfromCloudToMap[0] = roll;
  _transfromCloudToMap[1] = pitch;
  _transfromCloudToMap[2] = yaw;
  _transfromCloudToMap[3] = truthOdomTrans.getOrigin().x();
  _transfromCloudToMap[4] = truthOdomTrans.getOrigin().y();
  _transfromCloudToMap[5] = truthOdomTrans.getOrigin().z();
  // std::cout<<"roll: "<<roll<<" pitch: "<<pitch<<" yaw: "<<yaw<<std::endl;
  // std::cout<<"x: "<<_transfromCloudToMap[3]<<" y: "<<_transfromCloudToMap[4]<<" z: "<<_transfromCloudToMap[5]<<std::endl;
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

    // // rotate with x axis (roll).
    // x1 = _cloudIn->points[i].x;
    // y1 = cos(-_transfromCloudToMap[0])*_cloudIn->points[i].y + sin(-_transfromCloudToMap[0]) *_cloudIn->points[i].z;
    // z1 = -sin(-_transfromCloudToMap[0])*_cloudIn->points[i].y + cos(-_transfromCloudToMap[0]) *_cloudIn->points[i].z;
    //
    // // rotate with y axis (pitch).
    // z2 = cos(-_transfromCloudToMap[1])*z1 + sin(-_transfromCloudToMap[1])*x1;
    // y2 = y1;
    // x2 = -sin(-_transfromCloudToMap[1])*z1 + cos(-_transfromCloudToMap[1])*x1;
    //
    // // rotate with z axis (yaw).
    // cloudOut->points[i].x = cos(-_transfromCloudToMap[2])*x2 + sin(-_transfromCloudToMap[2])*y2 + _transfromCloudToMap[3];
    // cloudOut->points[i].y = -sin(-_transfromCloudToMap[2])*x2 + cos(-_transfromCloudToMap[2])*y2 + _transfromCloudToMap[4];
    // cloudOut->points[i].z = z2 + _transfromCloudToMap[5];

  }

  *_globalMap += *cloudOut;

  publishMap();
}

void PointcloudMapBuilder::publishMap(){
  sensor_msgs::PointCloud2 cloudMsgTemp;

  pcl::toROSMsg(*_globalMap, cloudMsgTemp);
  cloudMsgTemp.header.stamp = _cloudHeader.stamp;
  cloudMsgTemp.header.frame_id = _targetFrame;

  _pubGlobalMap.publish(cloudMsgTemp);
}
