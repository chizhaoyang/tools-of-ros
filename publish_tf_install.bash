#!/bin/bash

cd
rm -rf ros_tools_ws
source /opt/ros/indigo/setup.bash #Source environment just to be safe
mkdir -p ~/ros_tools_ws/src
cd ~/ros_tools_ws/src
catkin_init_workspace
cd ~/ros_tools_ws/
catkin_make

#intialize git repository in cataglyphis workspace
cd ~/ros_tools_ws/src
git init

#add cataglyphis remotes
git remote add tools https://github.com/chizhaoyang/tools-of-ros

#fetch codes from github

git fetch tools
git checkout master

cd ..
catkin_make

