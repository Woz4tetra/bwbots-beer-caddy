#!/usr/bin/env bash

echo "Running bw_bcause firmware monitor"

platformio device monitor --baud 9600 --port /dev/ttyTHS0
