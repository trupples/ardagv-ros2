#!/bin/bash

set -xe

USER=runner
WORKDIR="/home/$USER"

install_apt() {

echo "----- installing apt packages -----"

APT_PKGS="git \
build-essential \
libxml2 \
libzstd-dev \
libxml2-dev \
bison flex \
libcdk5-dev \
cmake \
libaio-dev \
libusb-1.0-0-dev \
libserialport-dev \
libxml2-dev \
libavahi-client-dev \
doxygen \
graphviz \
ros-humble-cv-bridge \
ros-humble-image-transport \
ros-humble-image-geometry \
ros-humble-pcl-ros \
ros-humble-compressed-image-transport \
ros-humble-ament-cmake \
v4l-utils \
gpiod
"
sudo rm /var/lib/dpkg/info/libc-bin*
sudo apt-get clean
sudo apt-get -y update
sudo apt-get install -y $APT_PKGS

}

build_libiio() {

echo "----- building libiio -----"
pushd $WORKDIR
git clone https://github.com/analogdevicesinc/libiio
cd libiio
git checkout libiio-v0
mkdir build && cd build
cmake ../
make && sudo make install
popd

}

build_tof(){

echo "----- building ToF SDK -----"
pushd $WORKDIR
mkdir tof_ws
cd tof_ws
sudo mv $WORKDIR/ToF .
sudo mv $WORKDIR/libs .
cd ToF
git config --global --add safe.directory /home/runner/tof_ws/ToF
git submodule init
sudo mkdir build && cd build
sudo cmake -DNXP=1 -DUSE_DEPTH_COMPUTE_OPENSOURCE=on -DWITH_-DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DCMAKE_BUILD_TYPE=Release ..
sudo make
sudo make install
popd

}

setup_ros2ws() {

echo "----- setup ROS2 WS -----"
pushd $WORKDIR
git clone https://github.com/trupples/ardagv-ros2
mkdir ros2_ws
cd ros2_ws
mkdir src
mv ../ardagv-ros2/ardagv* ./src
mv ../ardagv-ros2/adi_3dtof_adtf31xx ./src
sudo mv ../imu-ros2 ./src
popd

}

setup_user(){
echo "----- set up user groups and permissions -----"
#sudo ldconfig
sudo groupadd -g 994 gpio
sudo usermod -aG gpio runner
sudo usermod -aG video runner
sudo usermod -aG dialout runner
sudo usermod -aG netdev runner
}

case $1 in
	install_apt)
		install_apt
		;;
	build_libiio)
		build_libiio
		;;
	build_tof)
		build_tof
		;;
	setup_ros2ws)
		setup_ros2ws
		;;
	setup_user)
		setup_user
		;;
esac
		



