#!/usr/bin/env bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

ENABLE_JETSON_CLOCKS=${1:-}

echo "Running bwbots systemd service install script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
    BASE_INSTALL_DIR=/usr/local
fi

SCRIPT_NAME=run_roscore
ENV_SCRIPT_NAME=ros_env
SERVICE_NAME=roscore.service

LAUNCH_SCRIPT_NAME=run_roslaunch
STOP_LAUNCH_SCRIPT_NAME=stop_roslaunch
LAUNCH_SERVICE_NAME=roslaunch.service

JETSON_CLOCKS_NAME=jetson_clocks.service

chmod +x ${BASE_DIR}/${SCRIPT_NAME}
chmod +x ${BASE_DIR}/${ENV_SCRIPT_NAME}
chmod +x ${BASE_DIR}/${LAUNCH_SCRIPT_NAME}
chmod +x ${BASE_DIR}/${STOP_LAUNCH_SCRIPT_NAME}

BIN_INSTALL_DIR=${BASE_INSTALL_DIR}/bin
mkdir -p ${BIN_INSTALL_DIR}

echo "Copying service files"
SERVICE_ROOT_DIR=/etc/systemd/system/
mkdir -p ${SERVICE_ROOT_DIR}
cp ${BASE_DIR}/${SERVICE_NAME} ${SERVICE_ROOT_DIR}
cp ${BASE_DIR}/${LAUNCH_SERVICE_NAME} ${SERVICE_ROOT_DIR}
cp ${BASE_DIR}/${JETSON_CLOCKS_NAME} ${SERVICE_ROOT_DIR}

cp ${BASE_DIR}/${SCRIPT_NAME} ${BIN_INSTALL_DIR}
cp ${BASE_DIR}/${ENV_SCRIPT_NAME} ${BIN_INSTALL_DIR}
cp ${BASE_DIR}/${LAUNCH_SCRIPT_NAME} ${BIN_INSTALL_DIR}
cp ${BASE_DIR}/${STOP_LAUNCH_SCRIPT_NAME} ${BIN_INSTALL_DIR}

echo "Enabling systemd services"
systemctl daemon-reload
loginctl enable-linger $USER
systemctl enable ${SERVICE_NAME}
systemctl enable ${LAUNCH_SERVICE_NAME}

systemctl restart ${SERVICE_NAME}
systemctl restart ${LAUNCH_SERVICE_NAME}

if [ ! -z ${ENABLE_JETSON_CLOCKS} ]; then
    systemctl enable ${JETSON_CLOCKS_NAME}
    systemctl restart ${JETSON_CLOCKS_NAME}
fi

echo "bwbots systemd service installation complete"
