#!/bin/bash

set -e

mkdir -p /tmp
sudo chown -R 1000:1000 /tmp

# tbb
cd /tmp
git clone https://github.com/wjakob/tbb.git
cd tbb
git checkout 9e219e24fe223b299783200f217e9d27790a87b0
cd /tmp/tbb/build
cmake ..
make -j4
sudo make install

# apriltag
cd /tmp
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
git checkout 3e8e974d0d8d6ab318abf56d87506d15d7f2cc35
mkdir build
cd /tmp/apriltag/build
cmake ..
make -j4
sudo make install

# orocos_kinematics_dynamics
cd /tmp
git clone https://github.com/orocos/orocos_kinematics_dynamics.git
cd orocos_kinematics_dynamics
git checkout 5541147d4a220cab97d0ae1efa1aa860557d5c32
git submodule update --init
cd orocos_kdl
mkdir build
cd /tmp/orocos_kinematics_dynamics/orocos_kdl/build
cmake ..
make -j4
sudo make install

# python_orocos_kdl
mkdir -p mkdir ../../python_orocos_kdl/build || true
cd /tmp/orocos_kinematics_dynamics/python_orocos_kdl/build
cmake -D PYTHON_EXECUTABLE=/usr/bin/python3 \
    -D PYTHON_INCLUDE_DIR=/usr/include/python3.6 \
    -D PYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so \
    -D PYBIND11_PYTHON_VERSION=3 ..
make -j4
sudo make install

# g2o
cd /tmp
git clone https://github.com/RainerKuemmerle/g2o.git
cd /tmp/g2o
git checkout 20201223_git
mkdir build
cd /tmp/g2o/build
cmake ..
make -j4
sudo make install

# platformio
cd /tmp
download get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
export LC_ALL=en_US.utf-8
export LANG=en_US.utf-8
python get-platformio.py
sudo ln -s $HOME/.platformio/penv/bin/platformio /usr/local/bin

# nlopt
cd /tmp
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install

# clean up
sudo ldconfig
rm -r /tmp/* || true

echo "Built and installed all basic libraries"
