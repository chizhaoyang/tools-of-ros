#include <subscribe_data_to_txt/SubscribeDataToTxt.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "subscribe_data_to_txt_node");
	ros::NodeHandle n("~");

	SubscribeDataToTxt WritingTxt;

	WritingTxt.Initialize(n);

	ros::spin();
}