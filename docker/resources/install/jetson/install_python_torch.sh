#!/bin/bash

set -e

sudo chown -R 1000:1000 ${HOME}/.local

sudo -H python-torch -m pip install --no-cache-dir --upgrade pip setuptools

sudo -H python-torch -m pip install -r ./torch-requirements.txt

sudo -H python-torch -m pip uninstall -y kiwisolver
sudo -H python-torch -m pip install kiwisolver

echo "Installed python torch"
