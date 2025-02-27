#!/bin/bash

set -xe

IMU_ID=adis16470
USER=runner
WORKDIR=/home/$USER

set_rosdep(){

echo "----- set up ROSDEP -----"
pushd $WORKDIR/ros2_ws/
sudo rm  /etc/ros/rosdep/sources.list.d/20-default.list
export ROS_DISTRO=humble
sudo rosdep init
sudo rosdep fix-permissions
rosdep update
popd
}

build_ros2_pkgs() { 
echo "----- build ros2_pkgs -----"
pushd $WORKDIR/ros2_ws/
export DEVICE_ID=IMU_ID
export ROS_DISTRO=humble
source /opt/ros/humble/setup.sh
rosdep install --from-paths src/ardagv* --ignore-src -y
MAKEFLAGS='-j1 -l1' colcon build --packages-select imu_ros2 adi_3dtof_adtf31xx ardagv ardagv_crsf ardagv_motors
source install/setup.sh
popd
}

case $1 in
	set_rosdep)
		set_rosdep
		;;
	build_ros2_pkgs)
		build_ros2_pkgs
		;;
esac
