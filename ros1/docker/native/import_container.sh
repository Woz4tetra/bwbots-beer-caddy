#!/bin/bash
set -e

if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

cd /media/storage/docker

# echo "Importing bwbots image. This will take a while."
# docker load -i bwbots.tar.gz
echo "Decompressing docker_ros_ws"
tar -xvf bwbots_ws.tar.gz
echo "Done!"
