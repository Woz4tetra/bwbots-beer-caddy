#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")

if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

cd ${BASE_DIR}/../resources

docker buildx use bw-builder

for arch in arm64 ; do 
    docker buildx build \
    --platform linux/$arch \
    --output type=docker \
    -f ./Dockerfile \
    --tag bwbots-${arch}:latest .
done
