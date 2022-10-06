#
# this dockerfile roughly follows the 'Install ROS From Source' procedures from:
#   https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
#
ARG L4T_MAJOR_VERSION=35
ARG L4T_MINOR_VERSION=1
ARG L4T_PATCH_VERSION=0

FROM dustynv/ros:humble-pytorch-l4t-r${L4T_MAJOR_VERSION}.${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}

ENV L4T_MAJOR_VERSION=35
ENV L4T_MINOR_VERSION=1
ENV L4T_PATCH_VERSION=0

ENV ZED_SDK_MAJOR=3
ENV ZED_SDK_MINOR=7

ENV HOME=/home/nvidia
RUN mkdir -p /home/nvidia

ENV ROS_DOMAIN_ID=1
ENV ROS_NET_INTERFACE=wlan0

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

WORKDIR /

RUN apt-get update && apt-get install -y --no-install-recommends apt-utils

# 
# Setup interface
# 
COPY ./packages/cyclone-interface.xml /tmp/cyclone-interface.xml
RUN sed "s/INTERFACENAME/${ROS_NET_INTERFACE}/g" /tmp/cyclone-interface.xml > /cyclone-interface.xml
# ENV CYCLONEDDS_URI=file:///tmp/cyclone-interface.xml

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

# vision target dependency
WORKDIR /tmp
RUN git clone https://github.com/AprilRobotics/apriltag.git && \
    cd apriltag && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install && \
    cd ../.. && \
    rm -r apriltag

# make bwbots ros build workspace


ENV BW_ROS_ROOT=/home/nvidia/ros_ws
ENV BW_ROS_SRC=/home/nvidia/ros_ws/src
ENV BW_BUILD_WS=/home/nvidia/build_ws

RUN mkdir -p ${BW_ROS_SRC}
RUN mkdir -p ${BW_BUILD_WS}
RUN mkdir -p ${BW_BUILD_WS}/bwbots

#
# bwbots ROS dependencies
#
WORKDIR ${BW_ROS_SRC}

# RUN git clone https://github.com/Slamtec/sllidar_ros2.git
# RUN git clone https://github.com/ros-planning/navigation2.git -b humble
# RUN git clone https://github.com/ros-perception/laser_filters.git -b ros2
# RUN git clone https://github.com/ros-perception/vision_msgs.git -b ros2
RUN git clone https://github.com/ros2/common_interfaces.git -b humble
RUN git clone https://github.com/ros/joint_state_publisher.git -b ros2
RUN git clone https://github.com/ros/xacro.git -b ros2
# RUN git clone https://github.com/ros/bond_core.git -b ros2
# RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
# RUN git clone https://github.com/SteveMacenski/slam_toolbox.git -b humble
# RUN git clone https://github.com/ros-perception/vision_opencv.git -b humble
# RUN git clone https://github.com/ros-teleop/twist_mux -b rolling
# RUN git clone https://github.com/ros/diagnostics.git -b ros2-devel
RUN git clone https://github.com/RoverRobotics-forks/serial-ros2.git -b master
# RUN git clone https://github.com/ros-drivers/joystick_drivers.git -b ros2
# RUN git clone https://github.com/ros-tooling/topic_tools.git -b main
# RUN git clone https://github.com/ros-perception/image_transport_plugins.git -b humble
# RUN git clone https://github.com/frc-88/zed-ros2-wrapper.git -b tj2_detections
# RUN git clone https://github.com/rst-tu-dortmund/costmap_converter -b ros2
# RUN git clone https://github.com/rst-tu-dortmund/teb_local_planner.git -b ros2-master

WORKDIR ${BW_ROS_ROOT}

RUN export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/lib/python3.8/dist-packages/torch/share/cmake/Torch

RUN source /opt/ros/humble/install/local_setup.bash && \
    source /opt/ros/$ROS_DISTRO/install/setup.bash && \
    colcon build

#
# build bwbots ROS
#
COPY ./bwbots ${BW_ROS_SRC}/bwbots

RUN source /opt/ros/humble/install/local_setup.bash && \
    source /opt/ros/$ROS_DISTRO/install/setup.bash && \
    source ${HOME}/ros_ws/install/local_setup.bash && \
    source ${HOME}/ros_ws/install/setup.bash && \
    colcon build --packages-select-regex bw_*

#
# setup entry point
#

RUN rm /ros_entrypoint.sh
COPY ./packages/ros_entrypoint.sh /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR ${BW_ROS_ROOT}
