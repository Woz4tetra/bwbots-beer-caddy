#!/usr/bin/env bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

ifmetric wlan0 750
ifmetric eth0 0
