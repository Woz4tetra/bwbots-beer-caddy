#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")

if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

cd ..

apt install -y apt-transport-https ca-certificates curl \
	gnupg-agent software-properties-common

# Repo Key
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -

# Repo Itself
add-apt-repository -y \
  "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"

apt-get update
apt-get install -y docker docker-ce-cli containerd.io

usermod -aG docker $USER
systemctl enable docker

udevadm control --reload-rules && udevadm trigger

#Install Docker Compose
export DOCKER_COMPOSE_INSTALL_PATH=/usr/local/lib/docker/cli-plugins
export DOCKER_COMPOSE_URL=https://github.com/docker/compose/releases/download/v2.14.0/docker-compose-linux-armv7

mkdir -p ${DOCKER_COMPOSE_INSTALL_PATH}
curl -SL ${DOCKER_COMPOSE_URL} -o ${DOCKER_COMPOSE_INSTALL_PATH}/docker-compose
chmod +x ${DOCKER_COMPOSE_INSTALL_PATH}/docker-compose
curl -fL https://raw.githubusercontent.com/docker/compose-switch/master/install_on_linux.sh | sh

apt-get install -y nvidia-container-runtime
wget https://launchpad.net/ubuntu/+source/docker.io/20.10.2-0ubuntu1~18.04.2/+build/21335731/+files/docker.io_20.10.2-0ubuntu1~18.04.2_arm64.deb
dpkg -i docker.io_20.10.2-0ubuntu1~18.04.2_arm64.deb
rm docker.io_20.10.2-0ubuntu1~18.04.2_arm64.deb
apt install containerd=1.5.5-0ubuntu3~18.04.2


while IFS="" read -r p || [ -n "$p" ]
do
    printf '%s\n' "$p"

    if grep -Fq "$p"  /etc/apt/preferences ; then
        echo "command already exist in /etc/apt/preferences"
    else
        echo "Appending pin to /etc/apt/preferences"
        printf '%s\n' "$p" | tee -a /etc/apt/preferences > /dev/null
    fi
done < ${BASE_DIR}/freeze-docker-io.txt

cp ${BASE_DIR}/daemon.json /etc/docker/daemon.json

systemctl restart docker
