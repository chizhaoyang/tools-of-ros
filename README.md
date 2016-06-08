# tools-of-ros
a few tools for using ros

# publish_tf_node

bash publish_tf_install.bash

roscore

rosparam set use_sim_time true

rosbag play --pause --clock [rosbag file]

source ros_tools_ws/devel/setup.sh
rosrun publish_tf publish_tf_node

rviz
#add pointcloud2 to listen the lidar topic
