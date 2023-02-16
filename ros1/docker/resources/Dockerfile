#
# this dockerfile roughly follows the 'Install ROS From Source' procedures from:
#   https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
#

FROM dustynv/ros:noetic-pytorch-l4t-r32.6.1

ENV L4T_MAJOR_VERSION=32
ENV L4T_MINOR_VERSION=6
ENV L4T_PATCH_VERSION=1

ENV ZED_SDK_MAJOR=3
ENV ZED_SDK_MINOR=7

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && \
    apt-get install -y \
        nano tmux curl htop net-tools iproute2 iputils-ping gdb dumb-init \
        build-essential libtool autoconf zip unzip wget

# ---
# CMake
# ---

WORKDIR /tmp

RUN wget -O cmake.sh https://cmake.org/files/v3.23/cmake-3.23.1-linux-aarch64.sh
RUN mkdir /opt/cmake
RUN sh cmake.sh --prefix=/opt/cmake --exclude-subdir --skip-license
RUN mv /usr/bin/cmake /usr/bin/cmake-old
RUN ln -s /opt/cmake/bin/cmake /usr/bin/cmake

# ---
# PyTorch CMake
# ---

ENV LD_LIBRARY_PATH=/usr/local/lib/python3.6/dist-packages/torch/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/lib/python3.6/dist-packages/torch/share/cmake/Torch

# ---
# ZED
# ---

WORKDIR /

# This environment variable is needed to use the streaming features on Jetson inside a container
ENV LOGNAME root
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update || true
RUN apt-get install --no-install-recommends lsb-release less udev sudo apt-transport-https -y && \
    echo "# R${L4T_MAJOR_VERSION} (release), REVISION: ${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}" > /etc/nv_tegra_release ; \
    wget -q --no-check-certificate -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR_VERSION}.${L4T_MINOR_VERSION}/jetsons && \
    chmod +x ZED_SDK_Linux.run ; ./ZED_SDK_Linux.run silent skip_tools && \
    rm -rf /usr/local/zed/resources/* \
    rm -rf ZED_SDK_Linux.run

# This symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

# ---
# bwbots dependencies
# ---

RUN apt-get update && sudo apt-get install -y \
    libbullet-dev \
    libeigen3-dev \
    libsdl-image1.2-dev \
    libsdl-dev \
    libyaml-cpp-dev \
    libpcl-dev  \
    libvtk6-qt-dev  \
    libspnav-dev  \
    joystick  \
    liborocos-kdl-dev  \
    liborocos-kdl1.3  \
    libnetpbm10-dev  \
    libogg-dev  \
    libtheora-dev  \
    graphviz  \
    libgeographic-dev  \
    python3-pyproj  \
    ffmpeg  \
    libavcodec-dev  \
    libavformat-dev  \
    libavutil-dev  \
    libswscale-dev  \
    v4l-utils  \
    liburdfdom-headers-dev  \
    libtinyxml-dev  \
    liburdfdom-dev  \
    hddtemp  \
    lm-sensors  \
    python3-psutil  \
    libusb-1.0-0*  \
    libswscale-dev \
    libsuitesparse-dev \
    python3-termcolor \
    i2c-tools \
    python3-smbus \
    libceres-dev

RUN python3 -m pip install scipy==1.5.4 \
    shapely==1.6.4 \
    mido \
    pyperclip \
    dataclasses \
    py-trees==0.7.6 \
    tqdm

RUN apt-get install -y llvm-7*
RUN ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin
RUN python3 -m pip install Cython
RUN python3 -m pip install llvmlite==0.32.0
RUN python3 -m pip install numba==0.49.0

WORKDIR /tmp
RUN git clone https://github.com/wjakob/tbb.git
WORKDIR /tmp/tbb/build
RUN cmake ..
RUN make -j4 && make install

WORKDIR /tmp
RUN git clone https://github.com/AprilRobotics/apriltag.git && \
    cd apriltag && \
    mkdir build
WORKDIR /tmp/apriltag/build
RUN cmake ..
RUN make -j4 && make install

WORKDIR /tmp
RUN git clone https://github.com/orocos/orocos_kinematics_dynamics.git && \
    cd orocos_kinematics_dynamics && \
    git submodule update --init && \
    cd orocos_kdl && \
    mkdir build
WORKDIR /tmp/orocos_kinematics_dynamics/orocos_kdl/build
RUN cmake ..
RUN make -j4 && make install
RUN python3 -m pip install psutil
RUN mkdir ../../python_orocos_kdl/build
WORKDIR /tmp/orocos_kinematics_dynamics/python_orocos_kdl/build
RUN cmake -D PYTHON_EXECUTABLE=/usr/bin/python3 \
        -D PYTHON_INCLUDE_DIR=/usr/include/python3.6 \
        -D PYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so \
        -D PYBIND11_PYTHON_VERSION=3 ..
RUN make -j4 && make install

WORKDIR /tmp
RUN git clone https://github.com/RainerKuemmerle/g2o.git
WORKDIR /tmp/g2o
RUN git checkout 20201223_git
RUN mkdir build
WORKDIR /tmp/g2o/build
RUN cmake ..
RUN make -j4 && make install

WORKDIR /
RUN ldconfig
RUN rm -r /tmp/*

# ---
# bwbots
# ---

ENV USER=root
ENV HOME=/${USER}
WORKDIR ${HOME}

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${HOME}/ros_ws/src

ENV FLASK_ENV=development
ENV PYTHONPATH="${PYTHONPATH}:${ROS_WS_SRC}/bwbots/bw_tools"
ENV PYTHONIOENCODING=utf-8

COPY ./install ${HOME}/install
COPY ./configs ${HOME}/configs

RUN sudo mv ${HOME}/configs/zed/* /usr/local/zed/settings/
RUN sudo mv ${HOME}/install/preferences /etc/apt/preferences

RUN cat ${HOME}/install/bashrc >> ${HOME}/.bashrc
RUN rm ${HOME}/install/bashrc

COPY ./install/entrypoint.sh /entrypoint.sh
COPY ./install/launch.sh /launch.sh
COPY ./install/roscore.sh /roscore.sh

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
