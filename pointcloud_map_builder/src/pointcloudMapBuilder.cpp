#include <pointcloudMapBuilder.h>
//
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

pointcloudMapBuilder::pointcloudMapBuilder(){
  // get parameters
  if(!nh.getParam("topic/pointcloud", _pointcloudTopic)) ROS_ERROR("Failed to get param 'topic/pointcloud'");
  // if(!nh.getParam("topic/truthOdom", _truthOdomTopic)) ROS_ERROR("Failed to get param 'topic/truthOdom'");
  // if(!nh.getParam("frame/source", _sourceFrame)) ROS_ERROR("Failed to get param 'frame/source'");
  if(!nh.getParam("frame/target", _targetFrame)) ROS_ERROR("Failed to get param 'frame/target'");


  _cloudIn.reset(new pcl::PointCloud<PointType>())
  _globalMap.reset(new pcl::PointCloud<PointType>());

}

void pointcloudMapBuilder::pointcloudHandler(const sensor_msgs::PointCloud2COnstPtr& pointcloudMsg){
  _cloudHeader = pointcloudMsg->header;

  pcl::fromROSMsg(*pointcloudMsg, *_cloudIn);

  // Remove Nan points.
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_cloudIn, *_cloudIn, indeces);

  listenTFTrans();
}

void pointcloudMapBuilder::listenTFTrans(){

  // listen transform matrix from tf, M^T_C
  tf::StampedTransform truthOdomTrans;
  tf::TransformListener tfListener;
  try{
    tfListener.lookupTransform(_targetFrame, _cloudHeader.frame_id, _cloudHeader.stamp, truthOdomTrans);
  }
  catch (tf::TransformException &ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    continue;
  }

  double roll, pitch, yaw;
  geometry_mesgs::Quaternion geoQuat = truthOdomTrans.getRotation();
  // in this case, rotate seq: yaw pitch raw
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  _transfromCloudToMap[0] = roll;
  _transfromCloudToMap[1] = pitch;
  _transfromCloudToMap[2] = yaw;
  _transfromCloudToMap[3] = truthOdomTrans.getOrigin().x();
  _transfromCloudToMap[4] = truthOdomTrans.getOrigin().y();
  _transfromCloudToMap[5] = truthOdomTrans.getOrigin().z();

  transfromCloudToMap();
}

void pointcloudMapBuilder::transfromCloudToMap(){
  float x1, y1, z1, x2, y2, z2;
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCLoud<PointType>());

  size_t cloudSize;
  cloudSize = _cloudIn->points.size();
  for (size_t i=0; i<cloudSize; ++i){
    // rotate with z axis (yaw).
    x1 = cos(_transfromCloudToMap[2])*_cloudIn->points[i].x + sin(_transfromCloudToMap[2]) *_cloudIn->points[i].y;
    y1 = -sin(_transfromCloudToMap[2])*_cloudIn->points[i].x + cos(_transfromCloudToMap[2]) *_cloudIn->points[i].y;
    z1 = _cloudIn->points[i].z;

    // rotate with y axis (pitch).
    x2 = cos(_transfromCloudToMap[1])*x1 + sin(_transfromCloudToMap[1])*z1;
    y2 = y1;
    z2 = -sin(_transfromCloudToMap[1])*x1 + cos(_transfromCloudToMap[1])*z1;

    // rotate with x axis (roll).
    cloudOut[i].x = x2;
    cloudOut[i].y = cos(_transfromCloudToMap[0])*y2 + sin(_transfromCloudToMap[0])*z2;
    cloudOut[i].z = -sin(_transfromCloudToMap[0])*y2 + cos(_transfromCloudToMap[0])*z2;
  }
  *_globalMap += *cloudOut;

  publishMap();
}

void pointcloudMapBuilder::publishMap(){
  sensor_msgs::PointCloud2 cloudMsgTemp;

  pcl::toROSMsg(*_globalMap, cloudMsgTemp);
  cloudMsgTemp.header.stamp = _cloudHeader.stamp;
  cloudMsgTemp.header.frame_id = _targetFrame;

  _pubGlobalMap.publish(cloudMsgTemp);
}