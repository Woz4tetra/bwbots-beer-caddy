#!/usr/bin/env bash
BASE_DIR=$(realpath "$(dirname $0)")
BAG_PATH=$(realpath "./$1")
PARENT_DIR=$(dirname $BAG_PATH)

${BASE_DIR}/../bwbots/bw_tools/bw_tools/rosbag_to_file/convert.py -a -p ${BAG_PATH} -o ${PARENT_DIR}
