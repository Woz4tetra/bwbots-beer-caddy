FROM osrf/ros:noetic-desktop-full

RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils
RUN apt-get install -y git nano tmux curl htop net-tools iproute2 iputils-ping dumb-init
RUN apt-get install -y libsdl-image1.2-dev libsdl-dev python3-pip

RUN python3 -m pip install scipy==1.5.4 \
    shapely==1.6.4 \
    mido \
    pyperclip \
    dataclasses \
    tqdm

RUN apt-get install -y llvm-7*
RUN ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin
RUN python3 -m pip install Cython
RUN python3 -m pip install llvmlite==0.32.0
RUN python3 -m pip install numba==0.49.0

RUN ln -s /usr/bin/python3 /usr/bin/python

# ---
# bwbots dependencies
# ---

# generated using rosdep install --from-paths . -r --reinstall --simulate
RUN apt-get install -y --ignore-missing ros-noetic-rospy \
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
    ros-noetic-rosbridge-suite


# ---
# bwbots build environment
# ---

ENV USER=root
ENV HOME=/${USER}
WORKDIR ${HOME}

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${HOME}/ros_ws/src

ENV FLASK_ENV=development
ENV PYTHONPATH="${PYTHONPATH}:${ROS_WS_SRC}/bwbots/bw_tools"

COPY ./unity/docker/resources/install ${HOME}/install

RUN apt-get install -y xclip xvfb
RUN Xvfb :99 -screen 0 1280x720x16 &
RUN export DISPLAY=:99

# ---
# bwbots launch environment
# ---

RUN cat ${HOME}/install/bashrc >> ${HOME}/.bashrc
RUN rm ${HOME}/install/bashrc

COPY ./unity/docker/resources/install/entrypoint.sh /entrypoint.sh
COPY ./unity/docker/resources/install/launch.sh /launch.sh
COPY ./unity/docker/resources/install/roscore.sh /roscore.sh

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
