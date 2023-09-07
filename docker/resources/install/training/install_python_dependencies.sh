#!/bin/bash

set -e

sudo python -m pip install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu116 --no-cache-dir
sudo python -m pip install wandb --no-cache-dir

# yolov5
sudo mkdir -p /opt/yolov5
sudo chown -R 1000:1000 /opt/yolov5
cd /opt/yolov5
git clone https://github.com/ultralytics/yolov5.git
cd yolov5
sudo python -m pip install -r requirements.txt --no-cache-dir

sudo python -m pip install --no-cache-dir numpy==1.24.4

echo "Installed python dependencies"
