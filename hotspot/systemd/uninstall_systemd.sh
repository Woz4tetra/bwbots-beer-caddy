#!/usr/bin/env bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

BASE_DIR=$(realpath "$(dirname $0)")

ORGANIZATION=$(${BASE_DIR}/../../docker/get_organization)

echo "Running ${ORGANIZATION} hotspot service uninstall script"

SERVICE_NAME=${ORGANIZATION}-hotspot.service

SERVICE_ROOT_DIR=/etc/systemd/system/

rm ${SERVICE_ROOT_DIR}/${SERVICE_NAME}

echo "Disabling systemd services"
systemctl daemon-reload
systemctl stop ${SERVICE_NAME}
systemctl disable ${SERVICE_NAME}

echo "${ORGANIZATION} hotspot service uninstallation complete"
