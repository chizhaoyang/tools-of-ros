# tools-of-ros
a few tools for using ros

(before you try to use these tools, make sure you bash the install file to install all tools)

bash publish_tf_install.bash

# publish_tf_node

roscore

rosparam set use_sim_time true

rosbag play --pause --clock [rosbag file]

source ros_tools_ws/devel/setup.sh

rosrun publish_tf publish_tf_node

rviz
(add pointcloud2 to listen the lidar topic)

# rosbag_to_pcd_node

roscore

rosbag play --pause --clock [rosbag file]

source ros_tools_ws/devel/setup.sh

cd [path which you want to save all pcd files]

rosrun rosbag_to_pcd rosbag_to_pcd_node
 
