#!/usr/bin/env bash

echo "Running bw_bcause firmware monitor"

platformio device monitor --baud 9600 --port /dev/serial/by-id/usb-Teensyduino_USB_Serial_*
