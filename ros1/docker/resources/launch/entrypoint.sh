#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${DEP_ROS_WS_ROOT}/devel/setup.bash
source ${ROS_WS_ROOT}/devel/setup.bash
source ${HOME}/scripts/startup.sh

exec "$@"
