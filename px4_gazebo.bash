#!/bin/bash

#Gazebo world PATH
world_path=~/PX4-Autopilot/Tools/sitl_gazebo/worlds
#Input Gazebo Vehicle model name
model_path=~/PX4-Autopilot/Tools/sitl_gazebo/models/

world_default="empty"
vehicle_default="iris"

cd ~/PX4-Autopilot

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

roslaunch px4 mavros_posix_sitl.launch world:="${world_path}/${world}.world" sdf:=${model_path}/${vehicle_name}/${vehicle_name}.sdf fcu_url:="udp://:14540@192.168.1.36:14557"



