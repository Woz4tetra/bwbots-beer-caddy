#!/bin/bash

set -e

mkdir -p /tmp
sudo chown -R 1000:1000 /tmp

sudo apt-get install -y python3-venv
wget -O get-platformio.py https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py
python get-platformio.py
ln -s $HOME/.platformio/penv/bin/platformio /usr/local/bin

# clean up
sudo ldconfig
rm -r /tmp/* || true

echo "Built and installed all basic libraries"
