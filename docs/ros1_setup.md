
# ROS and Package Dependencies

I recommend running all of these commands inside of a tmux session in case of network dropouts:
- Create tmux session: `tmux new -s build`
- Hide tmux session: ctrl-B D
- Reattach tmux session: `tmux a -t build`

Download all packages to a home based directory:
- `mkdir ~/build_ws`
- `cd ~/build_ws`

Tip: list all CMake options: `cmake -LA | awk '{if(f)print} /-- Cache values/{f=1}'`

## Install g++
- `sudo apt update`
- `sudo apt-get install g++-8`

## Upgrade CMake
This is to fix an issue with the RTABmap build. Fix -lCUDA_cublas_device_LIBRARY-NOTFOUND issue:
- https://github.com/clab/dynet/issues/1457
- Install CMake version 3.12.2 or higher
<br>
<br>

- `sudo snap install cmake --classic`
- `sudo mv /usr/bin/cmake /usr/bin/cmake-old`
- `sudo ln -s /snap/bin/cmake /usr/bin`
- Close and reopen terminal for this to take effect

## Install TBB
- `cd ~/build_ws`
- `git clone https://github.com/wjakob/tbb.git`
- `cd tbb/build`
- `cmake ..`
- `make -j4`
- `sudo make install`

## Install numba

- `sudo apt install llvm-7*`
- `sudo ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin`
- `sudo -H python3 -m pip install Cython`
- `sudo -H python3 -m pip install llvmlite==0.32.0`
- `sudo -H python3 -m pip install numba==0.49.0`

## Install PyKDL

[Based on orocos's guide](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/python_orocos_kdl/INSTALL.md)

- `cd ~/build_ws`
- `git clone https://github.com/orocos/orocos_kinematics_dynamics.git`
- `cd orocos_kinematics_dynamics`
- `git submodule update --init`
<br><br>
- `cd orocos_kdl`
- `mkdir build && cd build`
- `cmake .. && make -j4`
- `sudo make install`
- `sudo -H python3 -m pip install psutil`
- `cd ../../python_orocos_kdl`
- `mkdir build && cd build`
- ```
    cmake -D PYTHON_EXECUTABLE=/usr/bin/python3 \
    -D PYTHON_INCLUDE_DIR=/usr/include/python3 \
    -D PYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.so \
    -D PYBIND11_PYTHON_VERSION=3 ..
  ```
- `make -j4`
- `sudo make install`
- `sudo ldconfig`
- `python3 ../tests/PyKDLtest.py`  The tests may fail. Make sure they at least run

## Install OpenCV 4

[Based on pyimagesearch's guide](https://www.pyimagesearch.com/2018/08/15/how-to-install-opencv-4-on-ubuntu/)

- `sudo apt-get install libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran python3-dev -y`
- cd ~/build_ws`
- `wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.5.0.zip`
- `wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.0.zip`
- `unzip opencv.zip && unzip opencv_contrib.zip`
- `mv opencv-4.5.0/ opencv && mv opencv_contrib-4.5.0/ opencv_contrib`
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
- `make -j4 && sudo make install` This will take several hours

## Block rosdep from installing apt’s opencv
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

## Install apriltag

- `cd ~/build_ws`
- `git clone https://github.com/AprilRobotics/apriltag.git`
- `cd apriltag && mkdir build && cd build`
- `cmake .. && make -j4 && sudo make install`

## Install g2o
- `cd ~/build_ws`
- `git clone https://github.com/RainerKuemmerle/g2o.git`
- `cd g2o`
- `git checkout 20201223_git`
- `mkdir build && cd build`
- `/snap/bin/cmake ..`  Higher version of cmake
- `make -j4`
- `sudo make install`

## Install yolov5 dependencies

## Install pytorch dependencies

- `cd ~/build_ws`
- `sudo -H python3 -m pip install future`
- `sudo -H python3 -m pip install -U wheel mock pillow`
- `sudo -H python3 -m pip install testresources`
- `sudo -H python3 -m pip install pybind11`
- `sudo -H python3 -m pip install cppy`
- `sudo -H python3 -m pip install "seaborn>=0.11.0" "pandas>=1.1.4" "scipy>=1.4.1" "matplotlib>=3.2.2" "tqdm"`
- `sudo apt-get install python3-pip libjpeg-dev libopenblas-dev libopenmpi-dev libomp-dev -y`

## Install pytorch from wheel
- `wget -O torch-1.10.0-cp36-cp36m-linux_aarch64.whl https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl`
- `sudo -H python3 -m pip install torch-1.10.0-cp36-cp36m-linux_aarch64.whl`

## Install pytorch vision
- `cd ~/build_ws`
- `git clone https://github.com/pytorch/vision`
- `cd vision`
- `git checkout v0.11.2`
    - if an error like this appears during the build: <br>`‘cached_cast’ is not a member of ‘at::autocast’` <br> checkout v0.7.0 instead
- `export MAKEFLAGS="-j2"`
- `sudo python3 setup.py install`
- `echo "export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/lib/python3.6/dist-packages/torch/share/cmake/Torch" >> ~/.bashrc`

## ZED SDK
- go to https://www.stereolabs.com/developers/release/
- Click SDK Downloads
- Find latest release for Jetpack 4.6/Jetson Xavier NX
- Copy the link
- `wget https://download.stereolabs.com/zedsdk/3.7/l4t32.6/jetsons`  Replace with your link
- `chmod +x jetsons`
- `./jetsons`
- Prompts:
  `Do you want to also install the static version of the ZED SDK (AI module will still require libsl_ai.so)` -> `y`
  `Do you want to install the AI module (required for Object detection and Neural Depth, recommended)` -> `y`
  `Do you want to enable maximum performance mode (recommended)? It provides optimal performance but increases power draw.` -> `n`  Do this manually later
  `Install samples (recommended)` -> `y`
  `Installation path: ` -> `/usr/local/zed/samples/`
  `Do you want to auto-install dependencies (recommended)` -> Press enter
  `Do you want to install the Python API (recommended)` -> `y`
  `Please specify your python executable: ` -> `python3`


# ROS Installation

- `cd ~/bwbots-beer-caddy/ros/install/robot_installation`
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

# bwbots-beer-caddy systemd install
- `cd ~/bwbots-beer-caddy/ros/systemd`
- `sudo ./install_systemd.sh`
- Verify installation:
    - `sudo systemctl status roscore.service`
    - `sudo systemctl status roslaunch.service`
