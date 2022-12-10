#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")

if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

cd ${BASE_DIR}/../resources
docker build -f ./Dockerfile -t bwbots:latest .
