#!/bin/bash

set -e

sudo rm /usr/bin/python || true
sudo rm /usr/bin/python3 || true
sudo ln -s /usr/local/bin/python3.9 /usr/bin/python-torch
sudo ln -s /usr/bin/python3.6 /usr/bin/python
sudo ln -s /usr/bin/python3.6 /usr/bin/python3

sudo -H python -m pip install --no-cache-dir --upgrade pip setuptools
sudo -H python -m pip install --no-cache-dir \
    scipy==1.5.4 \
    shapely==1.6.4 \
    dataclasses \
    flask==2.0.3 \
    psutil \
    tqdm \
    v4l2-fix

sudo apt-get update
sudo apt-get install -y llvm-7*
sudo ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin

sudo -H python -m pip install meson==0.47.0 --no-cache-dir
sudo -H python -m pip install Cython --no-cache-dir
sudo -H python -m pip install llvmlite==0.32.0 --no-cache-dir
sudo -H python -m pip install numba==0.49.0 --no-cache-dir

echo "Installed python dependencies"
