#!/bin/bash

set -e

groupadd -g 1000 ${USER}
useradd -r -u 1000 -m -s /bin/bash -g ${USER} -G dialout,plugdev,video,audio,sudo ${USER}
chown -R ${USER} ${HOME}
adduser ${USER} sudo
echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

echo "Setup user script complete"
