ARG ROS_DISTRO
ARG PROJECT_NAME
ARG ORGANIZATION

FROM ros:${ROS_DISTRO}-ros-core as base_image

RUN apt-get update && \
    apt-get install -y rsync
COPY --chown=1000:1000 \
    ./install/workstation/copy_select_from_base.sh \
    ./install/workstation/include_from_base.txt \
    /tmp/
RUN bash /tmp/copy_select_from_base.sh

FROM stereolabs/zed:4.0-devel-cuda11.8-ubuntu20.04

ARG ROS_DISTRO
ARG PROJECT_NAME
ARG ORGANIZATION

ENV ROS_DISTRO=${ROS_DISTRO}
ENV PROJECT_NAME=${PROJECT_NAME}
ENV ORGANIZATION=${ORGANIZATION}

ENV ZED_SDK_MAJOR=3 \
    ZED_SDK_MINOR=8 \
    DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash
SHELL ["/bin/bash", "-c"] 

# ---
# Basic tools
# ---

RUN sudo apt-get update && \
    sudo apt-get install -y apt-utils \
         git nano tmux curl wget htop net-tools iproute2 iputils-ping \
         gdb dumb-init rsync entr build-essential libtool autoconf zip unzip zstd

# ---
# Multistage copy
# ---

COPY --from=base_image /tmp/multistage_copy /tmp/multistage_copy
COPY --chown=1000:1000 ./install/training/copy_over_multistage.sh /opt/${ORGANIZATION}/install/training/
RUN bash /opt/${ORGANIZATION}/install/training/copy_over_multistage.sh


# ---
# Workstation steps
# ---

# ---
# User setup
# ---

ENV USER=${ORGANIZATION}
ENV HOME=/home/${USER}

RUN sudo mkdir -p /opt/${ORGANIZATION}/install && sudo chown -R 1000:1000 /opt/${ORGANIZATION}/
COPY --chown=1000:1000 ./install/setup_user.sh /opt/${ORGANIZATION}/install
RUN bash /opt/${ORGANIZATION}/install/setup_user.sh

COPY --chown=1000:1000 ./install/download /usr/bin

USER ${USER}

# ---
# PyTorch CMake
# ---

ENV LD_LIBRARY_PATH=/usr/local/lib/python3.6/dist-packages/torch/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}} \
    CMAKE_PREFIX_PATH=/usr/local/lib/python3.6/dist-packages/torch/share/cmake/Torch${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}


# ---
# Basic dependencies
# ---

COPY --chown=1000:1000 \
    ./install/workstation/install_apt_packages.sh \
    ./install/workstation/install_python_dependencies.sh \
    ./install/workstation/install_libraries.sh \
    /opt/${ORGANIZATION}/install/workstation/
RUN bash /opt/${ORGANIZATION}/install/workstation/install_apt_packages.sh && \
    bash /opt/${ORGANIZATION}/install/workstation/install_python_dependencies.sh && \
    bash /opt/${ORGANIZATION}/install/workstation/install_libraries.sh

# ---
# ROS dependency workspace
# ---

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws \
    DEP_ROS_WS_SRC=${HOME}/dep_ws/src

COPY --chown=1000:1000 ./install/rosdep_install.sh /opt/${ORGANIZATION}/install
COPY --chown=1000:1000 \
    ./install/workstation/install_ros_packages.sh \
    ./install/workstation/${PROJECT_NAME}_workstation.rosinstall \
    /opt/${ORGANIZATION}/install/workstation/
RUN bash /opt/${ORGANIZATION}/install/workstation/install_ros_packages.sh /opt/${ORGANIZATION}/install/workstation/${PROJECT_NAME}_workstation.rosinstall y

# ---
# Python extra packages
# ---

COPY --chown=1000:1000 \
    ./install/workstation/requirements.txt \
    ./install/workstation/install_python_extras.sh \
    /opt/${ORGANIZATION}/install/workstation/
RUN cd /opt/${ORGANIZATION}/install/workstation && bash ./install_python_extras.sh

# ---
# Training basic dependencies
# ---


COPY --chown=1000:1000 \
    ./install/training/install_apt_packages.sh \
    ./install/training/install_python_dependencies.sh \
    /opt/${ORGANIZATION}/install/training/
RUN bash /opt/${ORGANIZATION}/install/training/install_apt_packages.sh && \
    bash /opt/${ORGANIZATION}/install/training/install_python_dependencies.sh

# ---
# Training library dependencies
# ---

COPY --chown=1000:1000 \
    ./install/training/install_torchscript.sh \
    /opt/${ORGANIZATION}/install/training/
RUN bash -x /opt/${ORGANIZATION}/install/training/install_torchscript.sh

# ---
# Environment variables
# ---

ENV CMAKE_PREFIX_PATH=/usr/local/libtorch/share/cmake/Torch/${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}
ENV CMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs/${CMAKE_LIBRARY_PATH:+:${CMAKE_LIBRARY_PATH}}

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws
ENV DEP_ROS_WS_SRC=${HOME}/dep_ws/src

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${ROS_WS_ROOT}/src

ENV FLASK_ENV=development \
    PATH=${HOME}/.local/bin:/opt/${ORGANIZATION}/scripts${PATH:+:${PATH}} \
    PYTHONPATH=/opt/${ORGANIZATION}/${PROJECT_NAME}/src/bw_tools${PYTHONPATH:+:${PYTHONPATH}} \
    PYTHONIOENCODING=utf-8

# ---
# ROS dependency workspace
# ---

COPY --chown=1000:1000 \
    ./install/training/install_ros_packages.sh \
    ./install/training/${PROJECT_NAME}_training.rosinstall \
    /opt/${ORGANIZATION}/install/training/
RUN bash /opt/${ORGANIZATION}/install/training/install_ros_packages.sh /opt/${ORGANIZATION}/install/training/${PROJECT_NAME}_training.rosinstall


# ---
# launch environment
# ---

COPY --chown=1000:1000 ./install/client_bashrc ${HOME}/.bashrc

RUN chown 1000:1000 ${HOME} && \
    chown -R 1000:1000 ${HOME}/.ros

WORKDIR ${HOME}
USER ${USER}

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
