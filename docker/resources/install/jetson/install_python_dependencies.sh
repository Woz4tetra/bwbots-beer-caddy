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

sudo apt-get install -y llvm-7*
sudo ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin

sudo rm /usr/bin/python || true
sudo rm /usr/bin/python3 || true
sudo ln -s /usr/bin/python3.9 /usr/bin/python-torch
sudo ln -s /usr/bin/python3.8 /usr/bin/python
sudo ln -s /usr/bin/python3.8 /usr/bin/python3

sudo -H python -m pip install --no-cache-dir --upgrade pip setuptools
sudo -H python -m pip install --no-cache-dir \
    scipy==1.5.4 \
    shapely==1.6.4 \
    dataclasses \
    flask==2.0.3 \
    psutil \
    tqdm \
    v4l2-fix

sudo -H python -m pip install meson==0.47.0 --no-cache-dir
sudo -H python -m pip install Cython --no-cache-dir
sudo -H python -m pip install llvmlite==0.32.0 --no-cache-dir
sudo -H python -m pip install numba==0.49.0 --no-cache-dir

echo "Installed python dependencies"
