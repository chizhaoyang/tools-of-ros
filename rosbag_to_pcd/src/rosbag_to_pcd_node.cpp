#include "rosbag_to_pcd/rosbag_to_pcd.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"rosbag_to_pcd_node");

    Rosbag_To_Pcd rosbag_to_pcd;
	
	ros::spin();

	return 0;
}