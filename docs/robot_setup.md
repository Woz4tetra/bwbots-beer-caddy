
# SD card Installation

This guide assumes you're using a linux machine to run all these commands. You'll have to adapt these instructions if you're on another OS.

## From an SD card backup

### Loading a backup

[Based on this guide](https://www.jetsonhacks.com/2020/08/08/clone-sd-card-jetson-nano-and-xavier-nx/)

- [Download Jetson SD card image](https://i.kym-cdn.com/photos/images/newsfeed/002/203/505/797.png) WARNING this link doesn't work yet.
- Find disk path
- Run all commands as root: `sudo su`
- `parted -l`
  - GUI version: open "Disks"
- `umount /dev/sdd1`
  - Replace /dev/sdd1 with the partition that the OS auto mounts.
- `gunzip -c ./robeert.img.gz | dd of=/dev/sdd bs=64K status=progress`
  - Replace /dev/sdd with the top level disk name
  - WARNING: you can very easily destroy your disk if you select the wrong one. Please use caution here.


### Creating a backup

- `sudo dd if=/dev/sdd conv=sync,noerror bs=64K status=progress | gzip -c > ./robeert.img.gz`
  - Replace `/dev/sdd` with the top level disk name
  - WARNING: you can very easily destroy your disk if you select the wrong one. Please use caution here.

---

# Manual installation

If you don't have access to this SD card backup or need to start from scratch, follow these steps.

Unless otherwise stated, these commands are to be run on the Jetson.

I recommend running all of these commands inside of a tmux session in case of network dropouts:
- Create tmux session: `tmux new -s build`
- Hide tmux session: ctrl-B D
- Reattach tmux session: `tmux a -t build`

## Jetson initial setup

[Follow this guide from NVidia](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)

Setup options:
- username: ben
- password: s0mething
- hostname/computer name: robeert
- Log in automatically: Yes

After rebooting, try ssh: `ssh ben@chansey.local` <br>
Install openssh server if it doesn’t work: `sudo apt-get install openssh-server -y`

## If the Jetson's been set up already

### Change host name
- `sudo nano /etc/hostname`
- Replace with the robot's name
- `sudo nano /etc/hosts`
- Replace all instances of old hostname with the robot's name

### Change password

- `passwd`
- Type old password (probably nvidia)
- Change to s0mething

## Apt refresh + basic packages
- `sudo apt update`
- `sudo apt upgrade`
- `sudo apt autoremove`
- `sudo apt install nano tmux curl htop`
- `sudo reboot`

## SSH setup

If you haven't generated new SSH keys, [follow this guide](networking/ssh_instructions.md)

If you already have a key generated:
- Upload keys to `~/.ssh` on the Jetson (make the directory if it doesn’t exist)
- `cp robeert.pub authorized_keys`
- Optionally disable password login:
  - `sudo nano /etc/ssh/sshd_config`
  - Search for `#PasswordAuthentication yes`
  - Change to `PasswordAuthentication no`
  - Save and exit (ctrl-S ctrl-X)
- `sudo service ssh restart`
- Find the Jetson's IP with `ifconfig`
- Try to login with `ssh -i ~/.ssh/robeert nvidia@<your ip>`

## Disable wifi power saving 

[Based on this guide](https://unix.stackexchange.com/questions/269661/how-to-turn-off-wireless-power-management-permanently)

- `sudo nano /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf`
- Change:
```
[connection]
wifi.powersave = 3
```

to

```
[connection]
wifi.powersave = 2
```

- `sudo reboot`

## Add to sudo group
- `sudo usermod -aG sudo $USER`
- `sudo visudo`
- If vim opens,
  - Press a to enter edit mode
  - Add the following line to the file: <br>`nvidia  ALL=(ALL) NOPASSWD:ALL`
  - Press esc
  - Type `:x`
  - Press enter
- If nano opens,
  - Add the following line to the file: <br>`nvidia  ALL=(ALL) NOPASSWD:ALL`
  - Press ctrl-S then ctrl-X
- Log out with ctrl-D
- Log in and try `sudo su`
- If no password prompt appears, these steps worked

## Increase memory security limit
- `sudo nano /etc/security/limits.conf`
- File should look like this (increase all limits to 8GB):
  ```
  nvidia hard stack 8192
  nvidia soft stack 8192
  ubuntu hard stack 8192
  ubuntu soft stack 8192
  root hard stack 8192
  root soft stack 8192
  ```

## Upload code

Run this command on your local machine:

`~/bwbots-beer-caddy/install/upload.sh robeert.local ~/.ssh/robeert n`

## Python dependencies

- `export MAX_JOBS=4`
- `sudo -H python3 -m pip install --upgrade cython`
- `MAKEFLAGS="-j4" sudo -H python3 -m pip install --upgrade --force-reinstall numpy`
- `sudo -H python3 -m pip install setuptools --upgrade`
- `sudo -H python3 -m pip install --upgrade pip`
- Add this to `~/.bashrc`. This prevents `Illegal instruction (core dumped)` error when importing numpy:
  ```
  export OPENBLAS_CORETYPE=ARMV8
  ```

## Upload firmware

### Install platformio

[Based on this guide](https://docs.platformio.org/en/latest/core/installation.html#super-quick-mac-linux)

- `python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"`
- `sudo ln -s /home/$USER/.platformio/penv/bin/platformio /usr/local/bin`

### Main firmware upload

#### Compile

- `cd ~/bwbots-beer-caddy/firmware/bw_bcause`
- `./compile.sh`
- Packages should download
- Platformio should say "SUCCESS"

#### Upload
- sudo apt-get install libusb-0.1-4
- `wget https://www.pjrc.com/teensy/00-teensy.rules`
- `sudo mv 00-teensy.rules /etc/udev/rules.d/49-teensy.rules`
- `sudo usermod -aG dialout $USER`
- `sudo reboot`
- `cd ~/bwbots-beer-caddy/firmware/bw_bcause`
- `./upload.sh`
- Platformio should say "SUCCESS"

#### Troubleshooting

- If uploading gets stuck on `Waiting for Teensy device...`, you'll need to trigger an upload using the teensy app.
- For this you'll need a separate machine (with a screen)
- Follow the instructions here to download and install Teensyduino: https://www.pjrc.com/teensy/td_download.html
- Upload the blink project from the examples dropdown menu
- Try uploading using the above method again

### Load cell firmware upload

#### Compile

- `cd ~/bwbots-beer-caddy/firmware/bw_load_cell`
- `./compile.sh`
- Packages should download
- Platformio should say "SUCCESS"

#### Upload
- `cd ~/bwbots-beer-caddy/firmware/bw_load_cell`
- `./upload.sh`
- Platformio should say "SUCCESS"

## Configure serial port

### Disable serial console
- `sudo systemctl stop nvgetty`
- `sudo systemctl disable nvgetty`
- `sudo udevadm trigger`
- `sudo reboot`

## ROS 2 docker

- `sudo apt-get install nvidia-container-runtime`
- Edit/create the /etc/docker/daemon.json with content:
```
{
    "runtimes": {
        "nvidia": {
            "path": "/usr/bin/nvidia-container-runtime",
            "runtimeArgs": []
         } 
    },
    "default-runtime": "nvidia" 
}
```
- `sudo systemctl restart docker`
- `cd ~/bwbots-beer-caddy/ros2`
- `docker build -t bwbots:latest .`
