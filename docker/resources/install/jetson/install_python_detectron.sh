#!/bin/bash

set -e

cd /tmp
git clone https://github.com/facebookresearch/detectron2.git
cd detectron2
sudo -H python-torch setup.py install

echo "Installed python detectron"
