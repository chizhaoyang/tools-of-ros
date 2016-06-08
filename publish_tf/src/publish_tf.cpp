#include "publish_tf/publish_tf.h"

Publish_tf::Publish_tf()
{
	velodyne_points_Sub = node.subscribe("/velodyne_points", 1, &Publish_tf::getpointscallback, this);
}

void Publish_tf::getpointscallback(const sensor_msgs::PointCloud2& Pointcloud)
{
	child_frame = Pointcloud.header.frame_id;
	parent_frame = "map";

	quaternion.setRPY(0, 0, 0); //set roll pitch yaw from source frame to target frame (rotation)
	transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0)); //set translation, x, y, z from source frame to target frame
	transform.setRotation(quaternion);

	tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));
}


