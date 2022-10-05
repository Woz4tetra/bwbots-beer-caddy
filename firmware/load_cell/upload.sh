#!/usr/bin/env bash

echo "Running load_cell firmware upload script"

UPLOAD_PORT=${1:-/dev/ttyUSB0}

platformio run --target upload --upload-port=$UPLOAD_PORT
