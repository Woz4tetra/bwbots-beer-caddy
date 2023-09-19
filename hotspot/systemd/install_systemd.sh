#!/usr/bin/env bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

BASE_DIR=$(realpath "$(dirname $0)")

ORGANIZATION=$(${BASE_DIR}/../../docker/get_organization)

echo "Running ${ORGANIZATION} hotspot systemd service install script"

SERVICE_NAME=${ORGANIZATION}-hotspot.service

echo "Copying service files"
SERVICE_ROOT_DIR=/etc/systemd/system/
mkdir -p ${SERVICE_ROOT_DIR}
cp ${BASE_DIR}/${SERVICE_NAME} ${SERVICE_ROOT_DIR}

echo "Enabling systemd services"
systemctl daemon-reload
loginctl enable-linger $USER
systemctl enable ${SERVICE_NAME}

systemctl restart ${SERVICE_NAME}

echo "${ORGANIZATION} hotspot systemd service installation complete"
