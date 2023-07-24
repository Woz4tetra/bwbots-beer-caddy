#!/bin/bash

set -e

ROSINSTALL_PATH=$1

sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update
sudo apt-get install -y \
    python3-osrf-pycommon \
    python3-wstool \
    python3-catkin-pkg

mkdir -p ${DEP_ROS_WS_SRC}
cd ${DEP_ROS_WS_SRC}
wstool init .
wstool merge -t . ${ROSINSTALL_PATH}
wstool update -t .
/opt/${ORGANIZATION}/install/ros/patch_ros_packages.sh

cd ${DEP_ROS_WS_ROOT}
/opt/${ORGANIZATION}/install/rosdep_install.sh
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF

sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*

echo "Installed ROS packages"
