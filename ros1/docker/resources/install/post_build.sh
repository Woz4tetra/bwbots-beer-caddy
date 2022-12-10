#!/bin/bash
echo "ros workspace src: ${ROS_WS_SRC}"

cd ${ROS_WS_SRC}
/root/install/clone_ros_packages.sh

if [ ! -f ${ROS_WS_SRC}/PATCH_APPLIED ]; then
    /root/install/apply_patches.sh ${ROS_WS_SRC}
    touch ${ROS_WS_SRC}/PATCH_APPLIED
fi

source /opt/ros/${ROS_DISTRO}/setup.bash

cd ${ROS_WS_ROOT}
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y -r || true
catkin_make
