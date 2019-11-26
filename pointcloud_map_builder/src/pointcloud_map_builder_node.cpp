#include "pointcloud_map_builder/pointcloudMapBuilder.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "pointcloud_map_builder_node");

  ros::NodeHandle nh("~");

  PointcloudMapBuilder PMB(nh);

  ros::spin();
  return 0;
}
