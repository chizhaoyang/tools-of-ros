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
	if(!pu::Get("topic_name1", topic_name1_)) return false;
	if(!pu::Get("topic_name2", topic_name2_)) return false;
	if(!pu::Get("file_name1", file_name1_)) return false;
	if(!pu::Get("file_name2", file_name2_)) return false;

	writingfile1_.open(file_name1_.c_str());
	writingfile2_.open(file_name2_.c_str());

	return true;
}


bool SubscribeDataToTxt::RegisterCallbacks(const ros::NodeHandle& n){
	ros::NodeHandle nl(n);	// create local node

	// if need more than one topic, copy this line.
	sub_topic1_ = nl.subscribe(topic_name1_, 100, &SubscribeDataToTxt::WritingData1, this);
	sub_topic2_ = nl.subscribe(topic_name2_, 100, &SubscribeDataToTxt::WritingData2, this);

	return true;
}

// need modify this function to save specific variables
// void SubscribeDataToTxt::WritingData(const nav_msgs::Odometry& msg){

// 	writingfile_ << msg.header.stamp << " " << msg.header.frame_id << " " <<
// 	msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " <<
// 	msg.pose.pose.position.z << "\n";

// }

void SubscribeDataToTxt::WritingData1(const geometry_msgs::TransformStamped& msg){
	
	writingfile1_ << msg.header.stamp << " " << msg.header.frame_id << " " <<
	msg.child_frame_id << " " << msg.transform.translation.x << " " << 
	msg.transform.translation.y << " " << msg.transform.translation.z << "\n";
}

void SubscribeDataToTxt::WritingData2(const sensor_msgs::MagneticField& msg){

	writingfile2_ << msg.header.stamp << " " << msg.magnetic_field.x << " " <<
	msg.magnetic_field.y << " " << msg.magnetic_field.z << "\n";
}