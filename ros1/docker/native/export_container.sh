#!/bin/bash
set -e

if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

# echo "Exporting bwbots image. This will take a very long time."
# mkdir -p /media/storage/docker
# docker save bwbots:latest | gzip > /media/storage/docker/bwbots.tar.gz
echo "Compressing docker_ros_ws"
cd /media/storage
tar -cvf bwbots_ws.tar.gz docker_ros_ws
echo "Done!"
