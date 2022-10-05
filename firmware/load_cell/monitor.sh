#!/usr/bin/env bash

echo "Running bw_bcause firmware monitor"

platformio device monitor --baud 9600 --port /dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_*
