#!/usr/bin/env bash
BASE_DIR=$(realpath "$(dirname $0)")

packages=(
    https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
    https://github.com/frc-88/zed-ros-wrapper.git
)

branches=(
    main  # https://github.com/Unity-Technologies/ROS-TCP-Endpoint
    tj2_detections  # https://github.com/frc-88/zed-ros-wrapper.git
)

len=${#packages[@]}
for (( i=0; i<$len; i++ )); do
    git clone --recursive ${packages[i]} --branch ${branches[i]}
done
