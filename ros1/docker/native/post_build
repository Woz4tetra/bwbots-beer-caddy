#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")

mkdir -p ~/docker_ros_ws/src
mkdir -p ~/docker_ros_ws/devel
touch ~/docker_ros_ws/devel/setup.bash

cd ${BASE_DIR}/../resources

docker-compose -f docker-compose.post_build.yml up -d
docker logs -f post_build
