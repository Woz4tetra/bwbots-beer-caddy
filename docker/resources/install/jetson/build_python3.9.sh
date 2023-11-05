#!/bin/bash

set -e

sudo apt-get update

mkdir -p /tmp
sudo chown -R 1000:1000 /tmp
cd /tmp
sudo apt install -y zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev libsqlite3-dev libbz2-dev
wget https://www.python.org/ftp/python/3.9.1/Python-3.9.1.tar.xz
tar xvf Python-3.9.1.tar.xz Python-3.9.1/
mkdir build-python-3.9.1
cd build-python-3.9.1/
../Python-3.9.1/configure --enable-optimizations
make -j $(nproc)
sudo -H make altinstall

sudo rm -r /tmp/*
cd /
