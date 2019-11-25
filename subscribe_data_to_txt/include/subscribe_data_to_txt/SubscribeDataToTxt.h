/*	Subscribe topic from rostopic, and then save the specific data to file.txt

	Author: Chizhao Yang
*/

#include <iostream>
#include <fstream>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/MagneticField.h>

class SubscribeDataToTxt {
public:
	SubscribeDataToTxt();
	~SubscribeDataToTxt();

	bool Initialize(const ros::NodeHandle& n);

private:
	// Node initialization.
	bool LoadParameters(const ros::NodeHandle& n);

	bool RegisterCallbacks(const ros::NodeHandle& n);
	// Pose from vicon.
	void WritingData1(const geometry_msgs::TransformStamped& msg);
	// Magnetic information.
	void WritingData2(const sensor_msgs::MagneticField& msg);

	ros::Subscriber sub_topic1_;
	ros::Subscriber sub_topic2_;

	std::string name_;
	std::string topic_name1_;
	std::string file_name1_;
	std::string topic_name2_;
	std::string file_name2_;

	std::ofstream writingfile1_;
	std::ofstream writingfile2_;

};