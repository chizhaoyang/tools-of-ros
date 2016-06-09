#include "rosbag_to_pcd/rosbag_to_pcd.h"

Rosbag_To_Pcd::Rosbag_To_Pcd()
{
	sub_laser=node.subscribe("velodyne_points",1,&Rosbag_To_Pcd::getPointcloudCallback,this);

}

void Rosbag_To_Pcd::getPointcloudCallback(pcl::PointCloud<pcl::PointXYZI> const &pointcloud)
{
		std::stringstream ss;

		//ss << ros::Time::now();
		ss << "file" << count << ".pcd";

		pcl::io::savePCDFile(ss.str(),pointcloud);
		
		count++;

}


