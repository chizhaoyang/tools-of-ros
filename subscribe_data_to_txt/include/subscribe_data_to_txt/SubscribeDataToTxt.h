/*	Subscribe topic from rostopic, and then save the specific data to file.txt

	Author: Chizhao Yang
*/

#include <iostream>
#include <fstream>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

class SubscribeDataToTxt {
public:
	SubscribeDataToTxt();
	~SubscribeDataToTxt();

	bool Initialize(const ros::NodeHandle& n);

private:
	// Node initialization.
	bool LoadParameters(const ros::NodeHandle& n);

	bool RegisterCallbacks(const ros::NodeHandle& n);
	void WritingData(const nav_msgs::Odometry& msg);

	ros::Subscriber sub_topic_;

	std::string name_;
	std::string topic_name_;
	std::string file_name_;

	std::ofstream writingfile_;

};