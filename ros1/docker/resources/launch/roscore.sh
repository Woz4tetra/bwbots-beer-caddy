#!/bin/bash

source ${HOME}/scripts/set_master.sh ${ROS_MASTER_INTERFACE}
source /opt/ros/${ROS_DISTRO}/setup.bash

roscore
