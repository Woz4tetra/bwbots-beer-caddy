#!/bin/bash

set -e

sudo apt-get update
sudo apt-get install -y llvm-7*
sudo ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin

sudo rm /usr/bin/python || true
sudo ln -s /usr/bin/python3 /usr/bin/python

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
