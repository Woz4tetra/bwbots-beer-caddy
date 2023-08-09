#!/usr/bin/env bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

BASE_DIR=$(realpath "$(dirname $0)")

ORGANIZATION=$(${BASE_DIR}/../get_organization)

echo "Running ${ORGANIZATION} systemd service uninstall script"

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=/usr/local
fi

SERVICE_NAME=${ORGANIZATION}.service
SCRIPT_NAME=run_containers
STOP_SCRIPT_NAME=stop_container

BIN_INSTALL_DIR=${BASE_INSTALL_DIR}/bin

SERVICE_ROOT_DIR=/etc/systemd/system/

rm ${SERVICE_ROOT_DIR}/${SERVICE_NAME}

rm ${BIN_INSTALL_DIR}/${SCRIPT_NAME}
rm ${BIN_INSTALL_DIR}/${STOP_SCRIPT_NAME}

echo "Disabling systemd services"
systemctl daemon-reload
systemctl stop ${SERVICE_NAME}
systemctl disable ${SERVICE_NAME}

echo "${ORGANIZATION} systemd service uninstallation complete"
