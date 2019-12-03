#include <pointcloud_map_builder/tfToNavmsgs.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_to_navmsgs_node");

  ros::NodeHandle nh("~");

  TfToNavmsgs TTN(nh);

  while(ros::ok()){
    TTN.listenTFTrans();
    ros::spinOnce();
  }
  return 0;
}
