#!/bin/bash

set -e

sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update

sudo apt-get install -y --ignore-missing ros-noetic-rospy \
    ros-noetic-catkin \
    ros-noetic-tf \
    ros-noetic-rviz \
    ros-noetic-topic-tools \
    ros-noetic-joy \
    ros-noetic-roscpp \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-geometry2 \
    ros-noetic-tf2 \
    ros-noetic-nav-msgs \
    ros-noetic-navigation \
    ros-noetic-message-generation \
    ros-noetic-roslaunch \
    ros-noetic-message-runtime \
    ros-noetic-message-filters \
    ros-noetic-visualization-msgs \
    ros-noetic-image-transport \
    ros-noetic-vision-msgs \
    ros-noetic-pcl-msgs \
    ros-noetic-camera-info-manager \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-image-transport-plugins \
    ros-noetic-image-pipeline \
    ros-noetic-image-common \
    ros-noetic-marker-msgs \
    ros-noetic-usb-cam \
    libsdl-image1.2-dev \
    libsdl-dev \
    python3-pip \
    python3-tk \
    v4l-utils

sudo apt-get upgrade -y

sudo rm -rf /var/lib/apt/lists/*

echo "Installed all basic apt packages"
