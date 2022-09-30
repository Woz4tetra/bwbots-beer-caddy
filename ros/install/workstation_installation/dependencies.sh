#!/usr/bin/env bash
echo "Make sure you install ROS noetic first: https://wiki.ros.org/noetic/Installation/Ubuntu"
echo "and run: sudo rosdep init && rosdep update"

BASE_DIR=$(realpath "$(dirname $0)")

ROS_WS=$HOME/ros_ws
ROS_WS_SRC=${ROS_WS}/src
BUILD_WS=$HOME/build_ws

bash ${BASE_DIR}/install_apt_packages.sh ${BASE_DIR}

bash ${BASE_DIR}/install_ros_git_packages.sh ${BASE_DIR}
