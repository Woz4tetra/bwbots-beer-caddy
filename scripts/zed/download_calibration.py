#!/usr/bin/env python3
import os
import re
import subprocess
import urllib.request
pattern = r"\w\:  (.+)=(.+)"

output = subprocess.check_output("usb-devices").decode()

calibration_dir = os.path.expanduser("~/zed-resources")
if not os.path.isdir(calibration_dir):
    os.makedirs(calibration_dir)

devices = []
for section in output.split("\n\n"):
    info = {}
    for line in section.splitlines():
        if not line:
            continue
        match = re.search(pattern, line)
        if not match:
            continue
        key = match.group(1)
        value = match.group(2)
        info[key] = value
    devices.append(info)

for device in devices:
    manufacturer = device.get("Manufacturer", "")
    serial_number = device.get("SerialNumber", "")
    if manufacturer.lower() == "stereolabs":
        print(f"Found ZED camera. SN={serial_number}. Downloading calibration file.")
        urllib.request.urlretrieve(
            f"https://www.stereolabs.com/developers/calib/?SN={serial_number}", 
            os.path.join(calibration_dir, f"SN{serial_number}.conf")
        )

print("Done!")
