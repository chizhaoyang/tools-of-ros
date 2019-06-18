#include <subscribe_data_to_txt/SubscribeDataToTxt.h>
#include <parameter_utils/ParameterUtils.h>

namespace pu = parameter_utils;

SubscribeDataToTxt::SubscribeDataToTxt(){}

SubscribeDataToTxt::~SubscribeDataToTxt(){}

bool SubscribeDataToTxt::Initialize(const ros::NodeHandle& n){
	name_ = ros::names::append(n.getNamespace(), "SubscribeDataToTxt");

	if(!SubscribeDataToTxt::LoadParameters(n)){
		ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
		return false;
	}

	if(!SubscribeDataToTxt::RegisterCallbacks(n)){
		ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
		return false;
	}

	return true;
}

bool SubscribeDataToTxt::LoadParameters(const ros::NodeHandle& n){
	if(!pu::Get("topic_name", topic_name_)) return false;
	if(!pu::Get("file_name", file_name_)) return false;

	writingfile_.open(file_name_.c_str());

	return true;
}


bool SubscribeDataToTxt::RegisterCallbacks(const ros::NodeHandle& n){
	ros::NodeHandle nl(n);	// create local node

	// if need more than one topic, copy this line.
	sub_topic_ = nl.subscribe(topic_name_, 100, &SubscribeDataToTxt::WritingData, this);

	return true;
}

// need modify this function to save specific variables
void SubscribeDataToTxt::WritingData(const nav_msgs::Odometry& msg){

	writingfile_ << msg.header.stamp << " " << msg.header.frame_id << " " <<
	msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " <<
	msg.pose.pose.position.z << "\n";

}