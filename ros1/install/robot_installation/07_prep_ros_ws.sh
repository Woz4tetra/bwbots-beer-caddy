#!/usr/bin/env bash
BASE_DIR=$(realpath "$(dirname $0)")


SOURCE_COMMAND='source ${HOME}/ros_ws/devel/setup.bash
source ${HOME}/bwbots-beer-caddy/ros1/systemd/env.sh'

if ! grep -qz "$SOURCE_COMMAND" ~/.bashrc; then
    echo "Appending ros setup scripts to ~/.bashrc"
    echo "$SOURCE_COMMAND" | sudo tee -a ~/.bashrc > /dev/null
fi

WS_DIR=${HOME}/ros_ws/src/
mkdir -p ${WS_DIR}
SOURCE_DIR=$(realpath ${BASE_DIR}/../../bwbots)
ln -s ${SOURCE_DIR} ${WS_DIR}

echo "re-open this terminal session for bashrc to go into affect"
