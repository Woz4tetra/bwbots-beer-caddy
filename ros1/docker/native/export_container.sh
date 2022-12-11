#!/bin/bash
set -e

if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

docker images
echo "Paste in image ID to push: "
read TAG_IMAGE_ID
docker tag ${TAG_IMAGE_ID} woz4tetra/bwbots:latest
docker push woz4tetra/bwbots:latest

echo "Compressing docker_ros_ws"
cd ~
tar -cvf bwbots_ws.tar.gz docker_ros_ws
echo "Done!"
