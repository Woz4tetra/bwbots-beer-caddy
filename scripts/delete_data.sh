#!/bin/bash

set -e

RM_PATTERN="$1"

if [ -z "${RM_PATTERN}" ]; then
    echo "Removal pattern is empty! Exiting."
    exit 1
fi

rm -r ${ROS_WS_SRC}/${PROJECT_NAME}/bw_data/data/"${RM_PATTERN}"
