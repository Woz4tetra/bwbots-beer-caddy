#!/bin/bash

set -e

sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update

sudo apt-get install -y --ignore-missing ros-noetic-robot-localization \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-serial \
    ros-noetic-twist-mux \
    ros-noetic-cv-bridge \
    ros-noetic-image-geometry \
    ros-noetic-perception-pcl \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-gmapping \
    ros-noetic-laser-filters \
    ros-noetic-move-base \
    ros-noetic-teb-local-planner \
    ros-noetic-global-planner \
    ros-noetic-dwa-local-planner \
    ros-noetic-base-local-planner \
    ros-noetic-costmap-converter \
    ros-noetic-py-trees-ros \
    ros-noetic-py-trees-msgs \
    ros-noetic-py-trees \
    ros-noetic-rqt-py-trees \
    ros-noetic-rosbridge-suite \
    ros-noetic-ros-numpy

sudo apt-get upgrade -y

sudo rm -rf /var/lib/apt/lists/*

echo "Installed all basic apt packages"
