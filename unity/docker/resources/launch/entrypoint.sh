#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${DEP_ROS_WS_ROOT}/devel/setup.bash
source ${ROS_WS_ROOT}/devel/setup.bash
source ${HOME}/scripts/startup.sh

if [ ! -z ${HOST_MACHINE} ]; then
    source ${HOME}/scripts/set_client.sh ${HOST_MACHINE}
fi

exec "$@"
