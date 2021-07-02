#!/bin/bash

echo "ROS VERSION : $(rosversion -d)"

## ROS workspace initialization
### Initialize the project via git repo
catkin_make
mkdir -p src

git clone https://github.com/parham/ros_flir_spinnaker.git ./src
catkin_make
source devel/setup.bash

## Run the ROS NODE
roslaunch launch/combo_camera.launch
