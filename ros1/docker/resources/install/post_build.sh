#!/bin/bash
echo "ros workspace src: ${ROS_WS_SRC}"

cd ${ROS_WS_SRC}
/root/scripts/clone_ros_packages.sh

if [ ! -f ${ROS_WS_SRC}/PATCH_APPLIED ]; then
    /root/scripts/apply_patches.sh ${ROS_WS_SRC}
    touch ${ROS_WS_SRC}/PATCH_APPLIED
fi

source /opt/ros/${ROS_DISTRO}/setup.bash

cd ${ROS_WS_ROOT}
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y -r || true
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCATKIN_BLACKLIST_PACKAGES=""
