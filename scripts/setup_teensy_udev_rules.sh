wget -q https://www.pjrc.com/teensy/00-teensy.rules -O 49-teensy.rules
sudo mv 49-teensy.rules /etc/udev/rules.d/49-teensy.rules
sudo usermod -aG dialout $USER
sudo udevadm control --reload-rules && sudo udevadm trigger
