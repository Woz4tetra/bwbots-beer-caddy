#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")

if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

cd ${BASE_DIR}/../resources

apt-get install -y qemu binfmt-support qemu-user-static  # Install the qemu packages
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes  # This step will execute the registering scripts

docker run --rm -t arm64v8/ubuntu uname -m # Testing the emulation environment. Expected output: aarch64

docker run --privileged --rm tonistiigi/binfmt --install linux/amd64,linux/arm64

docker buildx create --name bw-builder --driver docker-container --bootstrap --use
docker buildx inspect
