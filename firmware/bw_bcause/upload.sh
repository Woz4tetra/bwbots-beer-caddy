#!/usr/bin/env bash

echo "Running dodobot firmware upload script"

platformio run --target upload --upload-port=/dev/serial/by-id/usb-Teensyduino_USB_Serial_*
