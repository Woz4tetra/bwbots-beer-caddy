#!/bin/bash

set -e

sudo apt-get update
sudo apt-get install -y llvm-10*
sudo ln -s /usr/lib/llvm-10/bin/llvm-config /usr/bin

sudo rm /usr/bin/python || true
sudo ln -s /usr/bin/python3 /usr/bin/python

sudo python -m pip install --no-cache-dir --upgrade pip setuptools
sudo python -m pip install --no-cache-dir \
    scipy==1.5.4 \
    shapely==1.6.4 \
    dataclasses \
    flask==2.0.3 \
    psutil \
    tqdm \
    v4l2-fix \
    numpy==1.24.4 \
    matplotlib==3.4.3

sudo python -m pip install Cython --no-cache-dir
sudo python -m pip install llvmlite==0.39.1 --no-cache-dir
sudo python -m pip install numba==0.56.4 --no-cache-dir

echo "Installed python dependencies"
