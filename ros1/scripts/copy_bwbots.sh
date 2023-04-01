#!/bin/bash

mkdir -p ${ROS_WS_SRC}
rm -r ${ROS_WS_SRC}/bwbots
cp -r ${HOME}/bwbots-beer-caddy/ros1/bwbots ${ROS_WS_SRC}/bwbots
