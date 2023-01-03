#!/bin/bash
echo "ros workspace src: ${ROS_WS_SRC}"

cd ${ROS_WS_SRC}
/root/install/clone_ros_packages.sh

source /opt/ros/${ROS_DISTRO}/setup.bash

cd ${ROS_WS_ROOT}
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y -r || true
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCATKIN_BLACKLIST_PACKAGES="bw_yolo;bw_zed;zed_nodelets;zed_ros;zed_wrapper"
