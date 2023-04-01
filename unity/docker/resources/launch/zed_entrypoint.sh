#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${HOME}/ros_ws/devel/setup.bash

if [ ! -z ${HOST_MACHINE} ]; then
    source ${HOME}/scripts/set_client.sh ${HOST_MACHINE}
fi

exec "$@"
