#!/usr/bin/env bash

echo "Running load_cell firmware upload script"

EXIT_CODE=255

while [ $EXIT_CODE -ne 0 ]; do
    platformio run --target upload --upload-port=/dev/serial/by-id/usb-Teensyduino_USB_Serial_*
    EXIT_CODE=$?
done

