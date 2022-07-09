#!/usr/bin/env bash

echo "Running firmware upload script"

platformio run --target upload --upload-port=/dev/serial/by-id/usb-Teensyduino_USB_Serial_6810740-if00
