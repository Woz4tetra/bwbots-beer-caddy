#
# this dockerfile roughly follows the 'Install ROS From Source' procedures from:
#   https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
#

FROM dustynv/ros:noetic-pytorch-l4t-r35.1.0

ENV L4T_MAJOR_VERSION=35
ENV L4T_MINOR_VERSION=1
ENV L4T_PATCH_VERSION=0

ENV ZED_SDK_MAJOR=3
ENV ZED_SDK_MINOR=7

ENV HOME=/home/nvidia
RUN mkdir -p /home/nvidia

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

WORKDIR /

RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y nano tmux curl htop net-tools iproute2

# ---
# ZED
# ---

#This environment variable is needed to use the streaming features on Jetson inside a container
ENV LOGNAME root
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update || true
RUN apt-get install --no-install-recommends lsb-release wget less udev sudo apt-transport-https build-essential cmake -y&& \
    echo "# R${L4T_MAJOR_VERSION} (release), REVISION: ${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}" > /etc/nv_tegra_release ; \
    wget -q --no-check-certificate -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR_VERSION}.${L4T_MINOR_VERSION}/jetsons && \
    chmod +x ZED_SDK_Linux.run ; ./ZED_SDK_Linux.run silent skip_tools && \
    rm -rf /usr/local/zed/resources/* \
    rm -rf ZED_SDK_Linux.run && \
    rm -rf /var/lib/apt/lists/*

#This symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

# ---
# bwbots dependencies
# ---

WORKDIR /tmp

RUN git clone https://github.com/wjakob/tbb.git && \
    cd tbb/build && \
    cmake .. && \
    make -j4 && \
    make install && \
    cd ../.. && \
    rm -r tbb

RUN apt-get update && \
    apt-get install -y llvm-7* && \
    ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin && \
    python3 -m pip install Cython && llvmlite==0.32.0 && numba==0.49.0

RUN apt-get install libeigen3-dev libsuitesparse-dev -y
# RUN git clone https://github.com/RainerKuemmerle/g2o.git && \
#     cd g2o && \
#     git checkout 20201223_git && \
#     mkdir build && \
#     cd build && \
#     cmake .. && \
#     make -j4 && \
#     make install && \
#     cd ../.. && \
#     rm -r g2o

RUN python3 -m pip install py_trees==0.7.6

RUN git clone https://github.com/AprilRobotics/apriltag.git && \
    cd apriltag && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install && \
    cd ../.. && \
    rm -r apriltag

RUN apt-get install -y libbullet-dev

# ---
# bwbots ROS dependencies
# ---

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${HOME}/ros_ws/src

RUN mkdir -p ${ROS_WS_SRC}
WORKDIR ${ROS_WS_SRC}

RUN mkdir /tmp/scripts
COPY --chown=1000:1000 ./scripts/clone_ros_packages.sh /tmp/scripts
COPY --chown=1000:1000 ./scripts/fix-image-pipeline.patch /tmp/scripts
COPY --chown=1000:1000 ./scripts/apply_patches.sh /tmp/scripts
RUN /tmp/scripts/clone_ros_packages.sh ${ROS_WS_SRC}
RUN /tmp/scripts/apply_patches.sh ${ROS_WS_SRC}
RUN rm -r /tmp/scripts

# ---
# build dependencies
# ---

WORKDIR ${ROS_WS_ROOT}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make

# ---
# setup entry point
# ---

RUN rm /ros_entrypoint.sh
COPY ./scripts/ros_entrypoint.sh /ros_entrypoint.sh
COPY ./scripts/launch.sh /launch.sh

WORKDIR ${ROS_WS_ROOT}

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/launch.sh"]
