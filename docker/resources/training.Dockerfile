ARG SOURCE_IMAGE
ARG PROJECT_NAME
ARG ORGANIZATION

FROM $SOURCE_IMAGE as workstation_bwbots

ENV PROJECT_NAME=${PROJECT_NAME}
ENV ORGANIZATION=${ORGANIZATION}

COPY --chown=1000:1000 \
    ./install/training/install_apt_packages.sh \
    ./install/training/install_python_dependencies.sh \
    /opt/${ORGANIZATION}/install/training/
RUN bash /opt/${ORGANIZATION}/install/training/install_apt_packages.sh && \
    bash /opt/${ORGANIZATION}/install/training/install_python_dependencies.sh

FROM stereolabs/zed:4.0-devel-cuda11.8-ubuntu20.04

ARG PROJECT_NAME
ARG ORGANIZATION

ENV PROJECT_NAME=${PROJECT_NAME}
ENV ORGANIZATION=${ORGANIZATION}

ENV L4T_MAJOR_VERSION=32 \
    L4T_MINOR_VERSION=6 \
    L4T_PATCH_VERSION=1 \
    ZED_SDK_MAJOR=3 \
    ZED_SDK_MINOR=8 \
    DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash
SHELL ["/bin/bash", "-c"] 

# ---
# Basic tools
# ---

RUN apt-get update && \
    apt-get install -y apt-utils \
        git nano tmux curl wget htop net-tools iproute2 iputils-ping gdb dumb-init rsync sudo

# ---
# User setup
# ---

ENV USER=${ORGANIZATION}
ENV HOME=/home/${USER}

RUN sudo mkdir -p /opt/${ORGANIZATION}/install && sudo chown -R 1000:1000 /opt/${ORGANIZATION}/
COPY --chown=1000:1000 ./install/setup_user.sh /opt/${ORGANIZATION}/install
RUN bash /opt/${ORGANIZATION}/install/setup_user.sh

USER ${USER}

# ---
# Multistage copy
# ---

COPY --from=workstation_bwbots / /tmp/workstation_${ORGANIZATION}
COPY --chown=1000:1000 ./install/training/copy_over_multistage.sh /opt/${ORGANIZATION}/install/training/
RUN bash /opt/${ORGANIZATION}/install/training/copy_over_multistage.sh

# ---
# Training dependencies
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

ENV ROS_DISTRO=noetic
ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${ROS_WS_ROOT}/src

ENV FLASK_ENV=development \
    PATH=${HOME}/.local/bin:/opt/${ORGANIZATION}/scripts${PATH:+:${PATH}} \
    PYTHONPATH=${ROS_WS_SRC}/${PROJECT_NAME}/bw_tools${PYTHONPATH:+:${PYTHONPATH}} \
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
