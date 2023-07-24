# Host dependencies

This guide assumes you're using a linux machine to run all these commands. You'll have to adapt these instructions if you're on another OS.

I recommend running all of these commands inside of a tmux session in case of network dropouts:

- Create tmux session: `tmux new -s build`
- Hide tmux session: ctrl-B D
- Reattach tmux session: `tmux a -t build`

## Jetson initial setup

[Follow this guide from NVidia](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)

Setup options:

- username: bw
- password: s0mething
- hostname/computer name: robeert
- Log in automatically: Yes

After rebooting, try ssh: `ssh bw@robeert.local` <br>
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

If you haven't generated new SSH keys, [follow this guide](ssh_instructions.md)

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

- `sudo systemctl restart network` This will kill your SSH session. Alternatively, you can run `sudo reboot`
- Have a display and keyboard on hand in case this goes wrong.

## Disable desktop

This reduces boot times by ~15 seconds.

https://forums.developer.nvidia.com/t/how-to-boot-jetson-nano-in-text-mode/73636

- `sudo systemctl set-default multi-user.target`

To re-enable the desktop:

- `sudo systemctl set-default graphical.target`

To start the desktop manually after logging into the CLI:

- `sudo systemctl start gdm3.service`

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

- `~/bwbots-beer-caddy/shortcuts/upload robeert n`

# Install dev rules

- Login into the Jetson: `~/bwbots-beer-caddy/shortcuts/login robeert`
- Install ZED rules: `~/bwbots-beer-caddy/scripts/setup_zed_udev_rules.sh`
- Install Teensy rules: `~/bwbots-beer-caddy/scripts/setup_teensy_udev_rules.sh`

# Install docker image

- Login into the Jetson: `~/bwbots-beer-caddy/shortcuts/login robeert`
- Pull container: `~/bwbots-beer-caddy/docker/jetson/pull_container`
- Install systemd service: `sudo ~/bwbots-beer-caddy/docker/systemd/install_systemd.sh`

# Configure serial port

## Disable serial console

- `sudo systemctl stop nvgetty`
- `sudo systemctl disable nvgetty`
- `sudo udevadm trigger`
- `sudo reboot`

# Install firmware

- Ensure container is running: `sudo systemctl status bwbots-beer-caddy.service`
- If not, troubleshoot. Restart with: `sudo systemctl restart bwbots-beer-caddy.service`
- Enter container: `~/bwbots-beer-caddy/docker/jetson/enter_container`
- Upload main firmware: `flash_bw_bcause`
- Upload load cell firmware: `flash_load_cell`

## Troubleshooting

- If uploading gets stuck on `Waiting for Teensy device...`, you'll need to trigger an upload using the teensy app.
- For this you'll need a separate machine (with a screen)
- Follow the instructions here to download and install Teensyduino: https://www.pjrc.com/teensy/td_download.html
- Upload the blink project from the examples dropdown menu
- Try uploading using the above method again
