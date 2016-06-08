#include "publish_tf/publish_tf.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "publish_tf_node");

	Publish_tf publish_tf;

	ros::spin();
	
	return 0;
}