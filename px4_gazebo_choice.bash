#!/bin/bash

#Gazebo world PATH
world_path=~/PX4-Autopilot/Tools/sitl_gazebo/worlds
#Input Gazebo Vehicle model name
model_path=~/PX4-Autopilot/Tools/sitl_gazebo/models/

world_default="empty"
vehicle_default="iris"


#Input Gazebo world name
echo "[List of Gazebo world]"
ls ${world_path}

read -p "Please Enter the World : " world
if [ -z ${world} ] ; then	
	world=${world_default}
fi

echo "[List of vehicle model]"
find $model_path -type d -name 'iris*' -exec basename {} \;
echo "typhoon"

echo "default = ${vehicle_default}"

read -p "Please Enter the Vehicle model : "  vehicle_name


if [ -z ${vehicle_name} ] ; then
	vehicle_name=${vehicle_default}	

elif [ ${vehicle_name} == "typhoon" ] ; then
	vehicle_name="typhoon_h480"

fi


cd ~/PX4-Autopilot
#DONT_RUN=1 make px4_sitl_default gazebo
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

#roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"

if [ ${vehicle_name} == "$typhoon_h480" ] ; then
	roslaunch px4 mavros_posix_sitl.launch vehicle:="typhoon_480" world:="${world_path}/${world}.world" sdf:=${model_path}/${vehicle_name}/${vehicle_name}.sdf fcu_url:="udp://:14540@192.168.1.36:14557"

else
	roslaunch px4 mavros_posix_sitl.launch world:="${world_path}/${world}.world" sdf:=${model_path}/${vehicle_name}/${vehicle_name}.sdf fcu_url:="udp://:14540@192.168.1.36:14557"

fi


