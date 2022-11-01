# Laptop workstation setup

- Instructions for Ubuntu 20

## Generate SSH keys
- Generate identifier keys: `ssh-keygen`. No passphrase. Default file directory.
```
Generating public/private rsa key pair.
Enter file in which to save the key (~/.ssh/id_rsa): 
Enter passphrase (empty for no passphrase): 
The key fingerprint is:
SHA256:?? ??@??
The key's randomart image is:
??
```

## Setup git

- `sudo apt update`
- `sudo apt install -y git tmux htop xclip`
- Enter identification (replace with account the keys are associated with):
  - `git config --global user.email "your@email.com"`
  - `git config --global user.name "yourusername"`

- Run the following command:
```
cat > ~/.ssh/config << 'EOF'
host github.com
  HostName github.com
  IdentityFile ~/.ssh/id_rsa
  User git
EOF
```

- Log into https://github.com/
- Go to settings

![alt text](images/GithubMenu.jpg "GithubMenu")
- Navigate to SSH and GPG keys

![alt text](images/GithubSSHkeys.jpg "GithubSSHkeys")

- Copy the contents of the public key file `id_rsa.pub`:
  - `xclip -sel c < ~/.ssh/id_rsa.pub`

- Click `Add SSH key`

- Paste contents of `id_rsa.pub` here

![alt text](images/GithubAddKey.jpg "GithubAddKey")

- Click `Add SSH key`

- `cd ~`
- Clone repository: `git clone git@github.com:Woz4tetra/bwbots-beer-caddy.git --recursive`
- `cd ./bwbots-beer-caddy`
- `git config pull.rebase false`

# Install ROS Noetic

- Follow ROS official instructions: http://wiki.ros.org/noetic/Installation/Ubuntu
  - Install `ros-noetic-desktop-full`

# Setup workspace

- `mkdir -p ~/ros_ws/src`
- `ln -s ~/bwbots-beer-caddy/ros1/bwbots/ ~/ros_ws/src`
- Install dependencies:
  - `cd ~/bwbots-beer-caddy/ros1/install/workstation_installation`
  - `sudo ./dependencies.sh`

# Connect to the robot via SSH

- Obtain the ssh keys `robeert` and `robeert.pub` (ping repo authors)
- `mv robeert ~/.ssh/`
- `mv robeert.pub ~/.ssh/`
- Log in: `ssh -i ~/.ssh/robeert nvidia@<your IP>`

# yolov5 installation for x86 linux systems

This is optional. Use this for training off the robot. Run all these commands on your local machine.

## Install CUDA if you haven’t already:
- https://developer.nvidia.com/cuda-downloads
- Download cuda for Ubuntu 20.04, deb
- `mkdir ~/build_ws`
- `cd ~/build_ws`
- `wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb`
- `sudo dpkg -i cuda-keyring_1.0-1_all.deb`
- `sudo apt-get update`
- `sudo apt-get -y install cuda`

## Install TensorRT
(Based on this guide)[https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#downloading]
- Download link: https://developer.nvidia.com/nvidia-tensorrt-download
(create a login if needed)
- Click `TensorRT 8` -> `TensorRT 8.4 GA Update 2` -> `TensorRT 8.4 GA Update 2 for Ubuntu 20.04 and CUDA 11.0, 11.1, 11.2, 11.3, 11.4, 11.5, 11.6 and 11.7 DEB local repo Package`
- `sudo dpkg -i nv-tensorrt-repo-ubuntu2004-cuda11.6-trt8.4.3.1-ga-20220813_1-1_amd64.deb`
- `sudo apt-get update`
- `sudo apt-get install tensorrt`

## Install pytorch
- `sudo -H pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu116`

## Install libtorch
- `cd ~/build_ws`
- `wget -O libtorch.zip https://download.pytorch.org/libtorch/cu116/libtorch-cxx11-abi-shared-with-deps-1.12.0%2Bcu116.zip`
- `unzip libtorch.zip`
- `sudo mv libtorch /usr/local/`
- `echo 'export CMAKE_PREFIX_PATH=/usr/local/libtorch/share/cmake/Torch/${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}' >> ~/.bashrc`
- `echo 'export CUDACXX=/usr/local/cuda/bin/nvcc' >> ~/.bashrc`
- Open a new terminal before building any cuda projects

## Install yolov5

- `cd ~/build_ws`
- `git clone git@github.com:frc-88/yolov5.git`
- `cd yolov5`
- `sudo -H python3 setup.py install`

### ZED SDK
- go to https://www.stereolabs.com/developers/release/
- Click SDK Downloads
- Find latest release for CUDA 11.X (11.0*->11.7), ZED SDK for Ubuntu 20
- Copy the link
- `wget https://download.stereolabs.com/zedsdk/3.7/cu117/ubuntu20`  Replace with your link
- `chmod +x ubuntu20`
- `./ubuntu20`
- Prompts:
  `Do you want to also install the static version of the ZED SDK (AI module will still require libsl_ai.so)` -> `y`
  `Do you want to install the AI module (required for Object detection and Neural Depth, recommended)` -> `y`
  `Install samples (recommended)` -> `y`
  `Installation path: ` -> `/usr/local/zed/samples/`
  `Do you want to auto-install dependencies (recommended)` -> Press enter
  `Do you want to install the Python API (recommended)` -> `y`
  `Please specify your python executable: ` -> `python3`

# Build workspace
- `cd ~/ros_ws`
- `catkin_make`

# Link rviz

- Open new terminal window
- Point local ROS session to remote master: `source ~/bwbots-beer-caddy/ros1/scripts/set_client.sh <your IP>`
- Open rviz: `rviz -d ~/bwbots-beer-caddy/ros1/src/bw_viz/rviz/standard.rviz`
- Make sure to run the `set_client.sh` script in every terminal you want to link to the robot. Feel free to create an alias for this command.

# Connect joystick

- Open new terminal window
- Point local ROS session to remote master: `source ~/bwbots-beer-caddy/ros1/scripts/set_client.sh <your IP>`
- Run local joystick node: `roslaunch bw_joystick bw_joystick.launch topic_name:=joy_remote device:=/dev/input/js0`
- Controls:
  - Left joystick up and down: forwards and backwards
  - Left joystick left and right: strafe left and right
  - Right joystick left and right: spin left and right
  - start + Right trigger: Enable motors
  - Left or Right trigger: Disable motors

# Build Jetson docker containers
- `sudo apt-get install qemu binfmt-support qemu-user-static`
- `docker run --rm --privileged multiarch/qemu-user-static --reset -p yes`
- 