#!/bin/bash
set -e
WS_TAR_PATH=$(realpath "$1")

if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

docker pull woz4tetra/bwbots
echo "Decompressing docker_ros_ws"
tar -xvf ${WS_TAR_PATH} -C /home/nvidia
echo "Done!"
