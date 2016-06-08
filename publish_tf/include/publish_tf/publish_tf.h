#include "ros/ros.h"
#include "ros/console.h"

#include "tf/transform_broadcaster.h"

#include "sensor_msgs/PointCloud2.h"

class Publish_tf
{
public:
	tf::TransformBroadcaster tfBroadcaster;
	Publish_tf();

protected:
	std::string parent_frame;
	std::string child_frame;

	ros::NodeHandle node;

	ros::Subscriber velodyne_points_Sub;

	
	tf::Transform transform;
	tf::Quaternion quaternion;

	void getpointscallback(const sensor_msgs::PointCloud2& Pointcloud);
};