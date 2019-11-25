// subscribe point cloud at current pose and transformation between current pose
// and the world frame;
// generate the point cloud map.
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filter/voxel_grid.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

typedef pcl::PointXYZI PointType;
class pointcloudMapBuilder{
public:
  pointcloudMapBuilder();
  ~pointcloudMapBuilder();

private:
  ros::Subscriber _subCloud;
  ros::Publisher _pubGlobalMap;
  
  std::string _pointcloudTopic;
  // std::string _sourceFrame;
  std::string _targetFrame;

  pcl::PointCloud<PointType>::Ptr _cloudIn;
  pcl::PointCloud<PointType>::Ptr _globalMap;
  pcl::PointCloud<PointType>::Ptr _globalMapDS;

  float _transfromCloudToMap[6];

  std_msgs::Header _cloudHeader;

}
