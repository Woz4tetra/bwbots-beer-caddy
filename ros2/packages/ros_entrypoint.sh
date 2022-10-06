#!/bin/bash
set -e

ros_env_setup="/opt/ros/$ROS_DISTRO/install/setup.bash"
echo "sourcing   $ros_env_setup"
source "$ros_env_setup"

source ${HOME}/ros_ws/install/local_setup.bash
source ${HOME}/ros_ws/install/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/$ROS_DISTRO

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

exec "$@"
