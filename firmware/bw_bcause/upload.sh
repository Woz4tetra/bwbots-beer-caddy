#!/usr/bin/env bash

echo "Running bw_bcause firmware upload script"

platformio run --target upload --upload-port=/dev/serial/by-id/usb-Teensyduino_USB_Serial_*
