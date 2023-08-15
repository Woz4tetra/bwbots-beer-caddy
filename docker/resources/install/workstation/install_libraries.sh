#!/bin/bash

set -e

mkdir -p /tmp
sudo chown -R 1000:1000 /tmp

# platformio
cd /tmp
download get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python get-platformio.py
sudo ln -s $HOME/.platformio/penv/bin/platformio /usr/local/bin

# clean up
sudo ldconfig
rm -r /tmp/* || true

echo "Built and installed all basic libraries"
