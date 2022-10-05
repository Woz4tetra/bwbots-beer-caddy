#!/usr/bin/env bash

echo "Running bw_bcause firmware upload script"

EXIT_CODE=255
UPLOAD_PORT=${1:-/dev/serial/by-id/usb-Teensyduino_USB_Serial_*}

while [ $EXIT_CODE -ne 0 ]; do
    platformio run --target upload --upload-port=$UPLOAD_PORT
    EXIT_CODE=$?
done

