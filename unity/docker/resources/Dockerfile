FROM osrf/ros:noetic-desktop-full

RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && \
    apt-get install -y git nano tmux curl htop net-tools iproute2 iputils-ping dumb-init gdb \
    libsdl-image1.2-dev libsdl-dev python3-pip wget python3-tk

RUN ln -s /usr/bin/python3 /usr/bin/python

RUN python -m pip install -U setuptools
RUN python -m pip install scipy==1.5.4 \
    shapely==1.6.4 \
    mido \
    pyperclip \
    dataclasses \
    numpy==1.22.4 \
    tqdm \
    requests \
    PyQt5==5.10.1

RUN apt-get install -y llvm-7*
RUN ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin
RUN python -m pip install Cython
RUN python -m pip install llvmlite==0.32.0
RUN python -m pip install numba==0.49.0

RUN python -m pip install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu116


RUN python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"
RUN ln -s $HOME/.platformio/penv/bin/platformio /usr/local/bin


# ---
# bwbots dependencies
# ---

# generated using rosdep install --from-paths . -r --reinstall --simulate
RUN apt-get update && apt-get install -y --ignore-missing ros-noetic-rospy \
    ros-noetic-catkin \
    ros-noetic-tf \
    ros-noetic-robot-localization \
    ros-noetic-rviz \
    ros-noetic-apriltag-ros \
    ros-noetic-topic-tools \
    ros-noetic-joy \
    ros-noetic-roscpp \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-geometry2 \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-serial \
    ros-noetic-tf2 \
    ros-noetic-nav-msgs \
    ros-noetic-twist-mux \
    ros-noetic-message-generation \
    ros-noetic-roslaunch \
    ros-noetic-message-runtime \
    ros-noetic-message-filters \
    ros-noetic-visualization-msgs \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-vision-msgs \
    ros-noetic-image-geometry \
    ros-noetic-camera-info-manager \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-image-transport-plugins \
    ros-noetic-perception-pcl \
    ros-noetic-pcl-msgs \
    ros-noetic-image-pipeline \
    ros-noetic-image-common \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-gmapping \
    ros-noetic-rplidar-ros \
    ros-noetic-laser-filters \
    ros-noetic-move-base \
    ros-noetic-teb-local-planner \
    ros-noetic-global-planner \
    ros-noetic-dwa-local-planner \
    ros-noetic-base-local-planner \
    ros-noetic-costmap-converter \
    ros-noetic-py-trees-ros \
    ros-noetic-py-trees-msgs \
    ros-noetic-py-trees \
    ros-noetic-rqt-py-trees \
    ros-noetic-rosbridge-suite \
    ros-noetic-marker-msgs \
    ros-noetic-ros-numpy

RUN sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-wstool


RUN python -m pip install wandb
RUN mkdir -p /opt/yolov5
WORKDIR /opt/yolov5
RUN git clone https://github.com/ultralytics/yolov5.git
RUN cd yolov5 && python -m pip install -r requirements.txt

WORKDIR /
RUN ldconfig

RUN apt-get clean

# ---
# User setup
# ---

ENV USER=root
ENV HOME=/${USER}
WORKDIR ${HOME}

# ---
# Build temporary workspace
# ---

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws
ENV DEP_ROS_WS_SRC=${HOME}/dep_ws/src

COPY ./unity/docker/resources/install ${HOME}/install

WORKDIR ${DEP_ROS_WS_SRC}
RUN wstool init .
RUN wstool merge -t . ${HOME}/install/bwbots_workstation.rosinstall
RUN wstool update -t .

RUN rosdep install -y --from-paths . --ignore-src --rosdistro noetic

RUN rm -r ${DEP_ROS_WS_SRC}/zed-ros-wrapper/zed_nodelets

WORKDIR ${DEP_ROS_WS_ROOT}
# https://github.com/ros/catkin/issues/863
RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF 
RUN catkin config --install
RUN catkin build

# ---
# ROS build environment
# ---

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${ROS_WS_ROOT}/src

ENV PYTHONPATH=${ROS_WS_SRC}/bw_tools:/opt/yolov5${PYTHONPATH:+:${PYTHONPATH}}

# ---
# bwbots launch environment
# ---

RUN cat ${HOME}/install/bashrc >> ${HOME}/.bashrc
RUN rm ${HOME}/install/bashrc

COPY ./unity/docker/resources/launch/entrypoint.sh /entrypoint.sh
COPY ./unity/docker/resources/launch/launch.sh /launch.sh
COPY ./unity/docker/resources/launch/roscore.sh /roscore.sh

WORKDIR ${HOME}

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
