
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
- `gunzip -c ./dodobot.img.gz | dd of=/dev/sdd bs=64K status=progress`
  - Replace /dev/sdd with the top level disk name
  - WARNING: you can very easily destroy your disk if you select the wrong one. Please use caution here.


### Creating a backup

- `sudo dd if=/dev/sdd conv=sync,noerror bs=64K status=progress | gzip -c > ./dodobot.img.gz`
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
- Try to login with `ssh -i ~/.ssh/chansey ben@chansey.local` (use the Jetson's IP address instead of chansey.local if that doesn't work)

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

## Upload code

Run this command on your local machine:

`~/bwbots-beer-caddy/install/upload.sh robeert.local ~/.ssh/robeert n`

## Python dependencies

- `sudo apt-get install python3.8-dev`
- `sudo rm /usr/bin/python3`
- `sudo rm /usr/bin/python`
- `sudo ln -s /usr/bin/python3.8 /usr/bin/python3`
- `sudo ln -s /usr/bin/python3.8 /usr/bin/python`
- `pip3 install --upgrade cython`
- `pip3 install --upgrade --force-reinstall numpy`

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

## ROS and Package Dependencies

I recommend running all of these commands inside of a tmux session in case of network dropouts:
- Create tmux session: `tmux new -s build`
- Hide tmux session: ctrl-B D
- Reattach tmux session: `tmux a -t build`

Download all packages to a home based directory:
- `mkdir ~/build_ws`
- `cd ~/build_ws`

Tip: list all CMake options: `cmake -LA | awk '{if(f)print} /-- Cache values/{f=1}'`

#### Upgrade CMake
This is to fix an issue with the RTABmap build. Fix -lCUDA_cublas_device_LIBRARY-NOTFOUND issue:
- https://github.com/clab/dynet/issues/1457
- Install CMake version 3.12.2 or higher
<br>
<br>

- `sudo apt remove --purge cmake`
- `sudo snap install cmake --classic`
- `echo "export PATH=${PATH}:/snap/bin" >> ~/.bashrc`
- Close and reopen terminal for this to take effect

### Install TBB
- `cd ~/build_ws`
- `git clone https://github.com/wjakob/tbb.git`
- `cd tbb/build`
- `cmake ..`
- `make -j3`
- `sudo make install`

### Install numba

- `sudo apt install llvm-7*`
- `sudo ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin`
- `sudo -H pip3 install Cython`
- `sudo -H pip3 install llvmlite==0.32.0`
- `sudo -H pip3 install numba==0.49.0`

### Install PyKDL

[Based on orocos's guide](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/python_orocos_kdl/INSTALL.md)

- `cd ~/build_ws`
- `git clone https://github.com/orocos/orocos_kinematics_dynamics.git`
- `cd orocos_kinematics_dynamics`
- `git submodule update --init`
<br><br>
- `cd orocos_kdl`
- `mkdir build && cd build`
- `cmake .. && make -j3`
- `sudo make install`
- `python3 -m pip install psutil`
- `cd ../../python_orocos_kdl`
- `mkdir build && cd build`
- ```
    cmake -D PYTHON_EXECUTABLE=/usr/bin/python3 \
    -D PYTHON_INCLUDE_DIR=/usr/include/python3.6 \
    -D PYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so \
    -D PYBIND11_PYTHON_VERSION=3 ..```
- `make -j3`
- `sudo make install`
- `sudo -H pip3 install psutil`
- `sudo ldconfig`
- `python3 ../tests/PyKDLtest.py`  The tests may fail. Make sure they at least run

### Install OpenCV 4

[Based on pyimagesearch's guide](https://www.pyimagesearch.com/2018/08/15/how-to-install-opencv-4-on-ubuntu/)

- `sudo apt-get install libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran python3-dev -y`
- cd ~/build_ws`
- `wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.4.0.zip`
- `wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.4.0.zip`
- `unzip opencv.zip && unzip opencv_contrib.zip`
- `mv opencv-4.4.0/ opencv && mv opencv_contrib-4.4.0/ opencv_contrib`
- `cd opencv && mkdir build && cd build`
- ```
  cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_CUDA=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D CUDNN_VERSION='8.0' \
    -D ENABLE_CXX11=ON \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -D PYTHON_EXECUTABLE=/usr/bin/python3 ..
  ```
- `make -j3 && sudo make install` This will take several hours

### Block rosdep from installing apt’s opencv
- `sudo nano /etc/apt/preferences`
- Paste the following contents:
    ```
    Package: python3-opencv
    Pin: release *
    Pin-Priority: -1

    Package: libopencv-dev
    Pin: release *
    Pin-Priority: -1
    ```

### Install apriltag

- `cd ~/build_ws`
- `git clone https://github.com/AprilRobotics/apriltag.git`
- `cd apriltag && mkdir build && cd build`
- `cmake .. && make -j3 && sudo make install`

### Install yolov5 dependencies

#### Install pytorch dependencies

- `cd ~/build_ws`
- `pip3 install "seaborn>=0.11.0" "pandas>=1.1.4" "thop" "scipy>=1.4.1" "matplotlib>=3.2.2" "tqdm"`
- `sudo apt-get install python3-pip libjpeg-dev libopenblas-dev libopenmpi-dev libomp-dev -y`
- `sudo -H pip3 install future`
- `sudo pip3 install -U --user wheel mock pillow`
- `sudo -H pip3 install testresources`
- `sudo -H pip3 install setuptools==58.3.0`
- `sudo -H pip3 install Cython`

#### Install pytorch from wheel
- `wget -O torch-1.10.0-cp36-cp36m-linux_aarch64.whl https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl`
- `sudo -H pip3 install torch-1.10.0-cp36-cp36m-linux_aarch64.whl`

#### Install pytorch from source

- `git clone --recursive https://github.com/pytorch/pytorch`
- `cd pytorch`
- `git checkout v1.10.2`
- `git submodule update`
- `mkdir build && cd build`
- `export MAX_JOBS=2`
  - cmake pulls this value into build commands run by make
- `cmake -DGLIBCXX_USE_CXX11_ABI=1 -DBUILD_SHARED_LIBS:BOOL=ON -DCMAKE_BUILD_TYPE:STRING=Release -DPYTHON_EXECUTABLE:PATH=`\`which python3\` `..`
    - If you get this error: “No CMAKE_CUDA_COMPILER could be found.” add nvcc to your path:
    - `echo "export CUDACXX=/usr/local/cuda/bin/nvcc" >> ~/.bashrc`
- `make -j2 && sudo make -j2 install`

#### Install pytorch vision
- `cd ~/build_ws`
- `git clone https://github.com/pytorch/vision`
- `cd vision`
- `git checkout v0.11.2`
    - if an error like this appears during the build: <br>`‘cached_cast’ is not a member of ‘at::autocast’` <br> checkout v0.7.0 instead
- `export MAKEFLAGS="-j2"`
- `sudo python3 setup.py install`
- `echo "export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${HOME}/.local/lib/python3.6/site-packages/torch/share/cmake/Torch" >> ~/.bashrc`
- `sudo apt-get install python3-dev`

## ROS Installation

- `cd ~/bwbots-beer-caddy/install/ros/robot_installation`
- `./00_ros_setup.sh`
- `./01_noetic_ws_prep.sh`
- `./02_noetic_ws_rosdep.sh` This may take a while
- `./03_noetic_ws_patch.sh`
- `./04_noetic_ws_install.sh` This will take a few hours
- `./05_append_env.sh`
- Close and reopen terminal
- `./06_build_packages_ws.sh y` y indicates we want to apply the patches. This will take a few hours
- Close and reopen terminal
- `./07_prep_ros_ws.sh`
- `./08_build_ros_ws.sh`
- `./09_install_python_libraries.sh`

## bwbots-beer-caddy systemd install
- `cd ~/bwbots-beer-caddy/ros/systemd`
- `sudo ./install_systemd.sh`
- Verify installation:
    - `sudo systemctl status roscore.service`
    - `sudo systemctl status roslaunch.service`
