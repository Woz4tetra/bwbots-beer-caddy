#!/usr/bin/env bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

echo "Running bwbots docker systemd service install script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
    BASE_INSTALL_DIR=/usr/local
fi

SERVICE_NAME=bwbots.service
SCRIPT_NAME=run_containers
STOP_SCRIPT_NAME=stop_main_container

BIN_INSTALL_DIR=${BASE_INSTALL_DIR}/bin
mkdir -p ${BIN_INSTALL_DIR}

echo "Copying service files"
SERVICE_ROOT_DIR=/etc/systemd/system/
mkdir -p ${SERVICE_ROOT_DIR}
cp ${BASE_DIR}/${SERVICE_NAME} ${SERVICE_ROOT_DIR}

cp ${BASE_DIR}/../${SCRIPT_NAME} ${BIN_INSTALL_DIR}
cp ${BASE_DIR}/../${STOP_SCRIPT_NAME} ${BIN_INSTALL_DIR}

echo "Enabling systemd services"
systemctl daemon-reload
loginctl enable-linger $USER
systemctl enable ${SERVICE_NAME}

systemctl restart ${SERVICE_NAME}

echo "bwbots docker systemd service installation complete"
