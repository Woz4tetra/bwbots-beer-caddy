#!/bin/bash

set -e

sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update

sudo apt-get install -y --ignore-missing \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-serial \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-perception-pcl \
    ros-${ROS_DISTRO}-amcl \
    ros-${ROS_DISTRO}-map-server \
    ros-${ROS_DISTRO}-gmapping \
    ros-${ROS_DISTRO}-laser-filters \
    ros-${ROS_DISTRO}-move-base \
    ros-${ROS_DISTRO}-teb-local-planner \
    ros-${ROS_DISTRO}-global-planner \
    ros-${ROS_DISTRO}-dwa-local-planner \
    ros-${ROS_DISTRO}-base-local-planner \
    ros-${ROS_DISTRO}-costmap-converter \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-ros-numpy \
    ros-${ROS_DISTRO}-rviz

sudo apt-get upgrade -y

echo "Installed all basic apt packages"
