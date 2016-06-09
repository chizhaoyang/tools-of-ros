#include <iostream>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <vector>
#include <cmath>
#include <stdlib.h>

using namespace std;

class Rosbag_To_Pcd
{
public:
	Rosbag_To_Pcd();
	
private:
	ros::NodeHandle node;
	ros::Subscriber sub_laser;
	int count = 0;
	void getPointcloudCallback(pcl::PointCloud<pcl::PointXYZI> const &pointcloud);
};