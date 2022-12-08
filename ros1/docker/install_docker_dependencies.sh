#!/bin/bash
sudo apt install -y apt-transport-https ca-certificates curl \
	gnupg-agent software-properties-common

# Repo Key
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

# Repo Itself
sudo add-apt-repository -y \
  "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io

sudo usermod -aG docker $USER
sudo systemctl enable docker

udevadm control --reload-rules && udevadm trigger

#Install Docker Compose
export DOCKER_COMPOSE_INSTALL_PATH=/usr/local/lib/docker/cli-plugins
export DOCKER_COMPOSE_URL=https://github.com/docker/compose/releases/download/v2.11.2/docker-compose-linux-x86_64

sudo mkdir -p ${DOCKER_COMPOSE_INSTALL_PATH}
sudo curl -SL ${DOCKER_COMPOSE_URL} -o ${DOCKER_COMPOSE_INSTALL_PATH}/docker-compose
sudo chmod +x ${DOCKER_COMPOSE_INSTALL_PATH}/docker-compose
curl -fL https://raw.githubusercontent.com/docker/compose-switch/master/install_on_linux.sh | sudo sh
sudo ln -s /usr/local/bin/compose-switch /usr/local/bin/docker-compose
