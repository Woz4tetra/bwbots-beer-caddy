#!/bin/bash

set -e

ROSINSTALL_PATH=$1

mkdir -p ${DEP_ROS_WS_SRC}
cd ${DEP_ROS_WS_SRC}
wstool merge -t . -ry ${ROSINSTALL_PATH}
wstool update -t .

cd ${DEP_ROS_WS_ROOT}
/opt/${ORGANIZATION}/install/rosdep_install.sh
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF

echo "Installed ROS training packages"
