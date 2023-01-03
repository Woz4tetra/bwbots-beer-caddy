#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${ROS_WS_ROOT}/devel/setup.bash

cd ${ROS_WS_SRC}/bwbots
PACKAGE_LIST=`ls -d bw_* | sed 's/\///g'`
PACKAGE_LIST=`echo "$PACKAGE_LIST" | tr '\n' ';'`
cd ${ROS_WS_ROOT}
catkin_make -DCATKIN_WHITELIST_PACKAGES=$PACKAGE_LIST -DCATKIN_BLACKLIST_PACKAGES="bw_yolo;bw_zed"
