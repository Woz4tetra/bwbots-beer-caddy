#!/bin/bash

set -e

ROSINSTALL_PATH=$1
PATCH_INCOMPATIBLE=$2


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

if [[ $PATCH_INCOMPATIBLE == "y" ]]; then
    cd ${DEP_ROS_WS_SRC}/zed-ros-wrapper
    git submodule update --init --recursive
    rm -r ${DEP_ROS_WS_SRC}/zed-ros-wrapper/zed_nodelets
else
    cd ${DEP_ROS_WS_SRC}/zed-ros-wrapper
    git submodule update --recursive
    git checkout -f
fi

cd ${DEP_ROS_WS_ROOT}
/opt/${ORGANIZATION}/install/rosdep_install.sh
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF

sudo rm -rf /var/lib/apt/lists/*

echo "Installed ROS workstation packages"