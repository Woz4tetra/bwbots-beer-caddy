#!/usr/bin/env bash

echo "Running drum_dispenser firmware monitor"

platformio device monitor --baud 115200 --port /dev/ttyUSB0
