#!/bin/bash

mkdir -p ${ROS_WS_SRC}
rsync -av ${HOME}/bwbots-beer-caddy/ros1/bwbots/* ${ROS_WS_SRC}/bwbots
