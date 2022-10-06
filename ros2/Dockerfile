#
# this dockerfile roughly follows the 'Install ROS From Source' procedures from:
#   https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
#
ARG L4T_BASE_IMAGE=l4t-base
ARG L4T_MAJOR_VERSION=32
ARG L4T_MINOR_VERSION=6
ARG L4T_PATCH_VERSION=1

FROM nvcr.io/nvidia/${L4T_BASE_IMAGE}:r${L4T_MAJOR_VERSION}.${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}

ARG ROS_PKG=ros_base
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV HOME=/home/bwbots
RUN mkdir /home/bwbots

#
# Set the default DDS middleware to cyclonedds
# https://github.com/ros2/rclcpp/issues/1335
#
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=1
ENV ROS_NET_INTERFACE=wlan0

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

WORKDIR /tmp

# change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

# set Python3 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1


# 
# add the ROS deb repo to the apt sources list
#
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils

RUN apt-get install -y --no-install-recommends \
        curl \
        wget \
        gnupg2 \
        lsb-release \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


# 
# install development packages
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libbullet-dev \
        libpython3-dev \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-pip \
        python3-numpy \
        python3-pytest-cov \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        python3-rosinstall-generator \
        libasio-dev \
        libtinyxml2-dev \
        libcunit1-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# install some pip packages needed for testing
RUN python3 -m pip install -U wheel
RUN python3 -m pip install -U \
        argcomplete \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        pytest-repeat \
        pytest-rerunfailures \
        pytest


# 
# install OpenCV (with CUDA)
#

#
# setup environment
#
ENV DEBIAN_FRONTEND=noninteractive
ENV CUDA_HOME="/usr/local/cuda"
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"

WORKDIR /opt


# 
# install OpenCV (with CUDA)
#
ARG OPENCV_URL=https://nvidia.box.com/shared/static/5v89u6g5rb62fpz4lh0rz531ajo2t5ef.gz
ARG OPENCV_DEB=OpenCV-4.5.0-aarch64.tar.gz

COPY scripts/opencv_install.sh /tmp/opencv_install.sh
RUN cd /tmp && ./opencv_install.sh ${OPENCV_URL} ${OPENCV_DEB}

# 
# upgrade cmake - https://stackoverflow.com/a/56690743
# this is needed to build some of the ROS2 packages
# note:  cmake is currently pinned to 3.22.3 because of https://github.com/pytorch/pytorch/issues/74955	 
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          software-properties-common \
          apt-transport-https \
          ca-certificates \
          gnupg \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean
                
RUN wget -qO - https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add - && \
    apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" && \
    apt-get update && \
    apt-get install -y --no-install-recommends --only-upgrade \
            cmake \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean
    
RUN cmake --version


#
# remove other versions of Python3
# workaround for 'Could NOT find Python3 (missing: Python3_NumPy_INCLUDE_DIRS Development'
#
RUN apt-get purge -y python3.9 libpython3.9* || echo "python3.9 not found, skipping removal" && \
    ls -ll /usr/bin/python*


# 
# download/build ROS from source
#
RUN mkdir -p ${ROS_ROOT}/src && \
    cd ${ROS_ROOT} && \
    # https://answers.ros.org/question/325245/minimal-ros2-installation/?answer=325249#post-id-325249
    rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ${ROS_PKG} \
        launch_xml \
        launch_yaml \
        launch_testing \
        launch_testing_ament_cmake \
        demo_nodes_cpp \
        demo_nodes_py \
        example_interfaces \
        camera_calibration_parsers \
        camera_info_manager \
        cv_bridge \
        v4l2_camera \
        vision_opencv \
        vision_msgs \
        image_geometry \
        image_pipeline \
        image_transport \
        compressed_image_transport \
        compressed_depth_image_transport \
        > ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    vcs import src < ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    # https://github.com/dusty-nv/jetson-containers/issues/181
    rm -r ${ROS_ROOT}/src/ament_cmake && \
    git -C ${ROS_ROOT}/src/ clone https://github.com/ament/ament_cmake -b ${ROS_DISTRO} && \
    # install dependencies using rosdep
    apt-get update && \
    cd ${ROS_ROOT} && \
    rosdep init && \
    rosdep update && \
    rosdep install -y -r \
          --ignore-src \
       --from-paths src \
      --rosdistro ${ROS_DISTRO} \
      --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    # build it!
    colcon build \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # remove build files
    rm -rf ${ROS_ROOT}/src && \
    rm -rf ${ROS_ROOT}/logs && \
    rm -rf ${ROS_ROOT}/build && \
    rm ${ROS_ROOT}/*.rosinstall
    
    
#
# fix broken package.xml in test_pluginlib that crops up if/when rosdep is run again
#
#   Error(s) in package '/opt/ros/foxy/build/pluginlib/prefix/share/test_pluginlib/package.xml':
#   Package 'test_pluginlib' must declare at least one maintainer
#   The package node must contain at least one "license" tag
#
#RUN TEST_PLUGINLIB_PACKAGE="${ROS_ROOT}/build/pluginlib/pluginlib_enable_plugin_testing/install/test_pluginlib__test_pluginlib/share/test_pluginlib/package.xml" && \
#    sed -i '/<\/description>/a <license>BSD<\/license>' $TEST_PLUGINLIB_PACKAGE && \
#    sed -i '/<\/description>/a <maintainer email="michael@openrobotics.org">Michael Carroll<\/maintainer>' $TEST_PLUGINLIB_PACKAGE && \
#    cat $TEST_PLUGINLIB_PACKAGE


#
# setup entrypoint
#
COPY ./packages/ros_entrypoint.sh /ros_entrypoint.sh

RUN sed -i \
    's/ros_env_setup="\/opt\/ros\/$ROS_DISTRO\/setup.bash"/ros_env_setup="${ROS_ROOT}\/install\/setup.bash"/g' \
    /ros_entrypoint.sh && \
    cat /ros_entrypoint.sh

RUN echo 'source ${ROS_ROOT}/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

WORKDIR /tmp

# 
# Setup interface
# 
COPY ./packages/cyclone-interface.xml /tmp/cyclone-interface.xml
RUN sed "s/INTERFACENAME/${ROS_NET_INTERFACE}/g" /tmp/cyclone-interface.xml > /cyclone-interface.xml



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

# make bwbots ros build workspace


ENV BW_ROS_ROOT=/home/bwbots/ros_ws
ENV BW_ROS_SRC=/home/bwbots/ros_ws/src
ENV BW_BUILD_WS=/home/bwbots/build_ws

RUN mkdir ${BW_ROS_SRC}
RUN mkdir ${BW_BUILD_WS}

COPY ./bwbots ${BW_ROS_SRC}

# vision target dependency
RUN git clone https://github.com/AprilRobotics/apriltag.git && \
    cd apriltag && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install

# teb_local_planner dependency
RUN git clone https://github.com/RainerKuemmerle/g2o.git && \
    cd g2o && \
    git checkout 20201223_git && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install

# pytorch dependencies
RUN pip3 install --no-cache-dir setuptools wheel
RUN pip3 install --no-cache-dir --verbose numpy

#
# PyTorch (for JetPack 4.4 DP)
#
#  PyTorch v1.2.0 https://nvidia.box.com/shared/static/lufbgr3xu2uha40cs9ryq1zn4kxsnogl.whl (torch-1.2.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.3.0 https://nvidia.box.com/shared/static/017sci9z4a0xhtwrb4ps52frdfti9iw0.whl (torch-1.3.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.4.0 https://nvidia.box.com/shared/static/c3d7vm4gcs9m728j6o5vjay2jdedqb55.whl (torch-1.4.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.5.0 https://nvidia.box.com/shared/static/3ibazbiwtkl181n95n9em3wtrca7tdzp.whl (torch-1.5.0-cp36-cp36m-linux_aarch64.whl)
#
ARG PYTORCH_URL=https://nvidia.box.com/shared/static/lufbgr3xu2uha40cs9ryq1zn4kxsnogl.whl
ARG PYTORCH_WHL=torch-1.2.0-cp36-cp36m-linux_aarch64.whl

RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${PYTORCH_URL} -O ${PYTORCH_WHL} && \
    pip3 install --no-cache-dir --verbose ${PYTORCH_WHL} && \
    rm ${PYTORCH_WHL}


#
# torchvision 0.4
#
ARG TORCHVISION_VERSION=v0.4.0
ARG TORCH_CUDA_ARCH_LIST="5.3;6.2;7.2;8.7"

RUN printenv && echo "torchvision version = $TORCHVISION_VERSION" && echo "TORCH_CUDA_ARCH_LIST = $TORCH_CUDA_ARCH_LIST"

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		  git \
		  build-essential \
            libjpeg-dev \
		  zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN git clone https://github.com/pytorch/vision torchvision && \
    cd torchvision && \
    git checkout ${TORCHVISION_VERSION} && \
    python3 setup.py install && \
    cd ../ && \
    rm -rf torchvision

# note:  this was used on older torchvision versions (~0.4) to workaround a bug,
#        but has since started causing another bug as of torchvision 0.11.1
# ARG PILLOW_VERSION=pillow<7    
# pip3 install --no-cache-dir "${PILLOW_VERSION}"

		  
# typically --only-upgrade is used in the apt install, but cmake wasn't previously installed in this container	
# note: skipping this way for now, because having trouble pinning it to specific version (see below)
#RUN wget -qO - https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add - && \
#    apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" && \
#    apt-get update && \
#    apt-cache policy cmake && \
#    apt-get install -y --no-install-recommends \
#            cmake \
#    && rm -rf /var/lib/apt/lists/* \
#    && apt-get clean


# patch for https://github.com/pytorch/pytorch/issues/45323
RUN PYTHON_ROOT=`pip3 show torch | grep Location: | cut -d' ' -f2` && \
    TORCH_CMAKE_CONFIG=$PYTHON_ROOT/torch/share/cmake/Torch/TorchConfig.cmake && \
    echo "patching _GLIBCXX_USE_CXX11_ABI in ${TORCH_CMAKE_CONFIG}" && \
    sed -i 's/  set(TORCH_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=")/  set(TORCH_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=0")/g' ${TORCH_CMAKE_CONFIG}

    
#
# torchaudio
#
ARG TORCHAUDIO_VERSION

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		  pkg-config \
		  libffi-dev \
		  libsndfile1 \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN pip3 install --no-cache-dir scikit-build && \
    pip3 install --no-cache-dir ninja && \
    pip3 install --no-cache-dir --verbose pysoundfile

# note:  see https://github.com/pytorch/audio/issues/2295 for the reason for the sed commands
RUN git clone --recursive -b ${TORCHAUDIO_VERSION} https://github.com/pytorch/audio torchaudio && \
    cd torchaudio && \
    sed -i 's#  URL https://zlib.net/zlib-1.2.11.tar.gz#  URL https://zlib.net/zlib-1.2.12.tar.gz#g' third_party/zlib/CMakeLists.txt || echo "failed to patch torchaudio/third_party/zlib/CMakeLists.txt" && \
    sed -i 's#  URL_HASH SHA256=c3e5e9fdd5004dcb542feda5ee4f0ff0744628baf8ed2dd5d66f8ca1197cb1a1#  URL_HASH SHA256=91844808532e5ce316b3c010929493c0244f3d37593afd6de04f71821d5136d9#g' third_party/zlib/CMakeLists.txt || echo "failed to patch torchaudio/third_party/zlib/CMakeLists.txt" && \
    BUILD_SOX=1 python3 setup.py install && \
    cd ../ && \
    rm -rf torchaudio


#
# PyCUDA
#
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"
RUN echo "$PATH" && echo "$LD_LIBRARY_PATH"

RUN pip3 install --no-cache-dir --verbose pycuda six

# for faster particle filters
RUN apt-get install llvm-7*
RUN ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin
RUN python3 -m pip install -U --no-cache-dir Cython
RUN python3 -m pip install -U llvmlite==0.32.0
RUN python3 -m pip install -U numba==0.49.0

#
# bwbots ROS dependencies
#
WORKDIR ${BW_ROS_SRC}

RUN git clone https://github.com/Slamtec/sllidar_ros2.git
RUN git clone https://github.com/ros/filters.git && cd filters && git checkout 2.1.0 && cd ..
RUN git clone https://github.com/ros/angles.git -b humble-devel
RUN git clone https://github.com/ros-perception/laser_filters.git -b ros2
RUN git clone https://github.com/ros-perception/vision_msgs.git -b ros2
RUN git clone https://github.com/ros/joint_state_publisher.git -b ros2
RUN git clone https://github.com/ros/xacro.git -b ros2
RUN git clone https://github.com/ros-planning/navigation2.git -b humble
RUN git clone https://github.com/ompl/ompl.git -b 1.5.0
RUN git clone https://github.com/ros/bond_core.git -b ros2
RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
RUN git clone https://github.com/SteveMacenski/slam_toolbox.git -b humble
RUN git clone https://github.com/ros-perception/vision_opencv.git -b humble
RUN git clone https://github.com/ros-teleop/twist_mux -b rolling
RUN git clone https://github.com/ros/diagnostics.git -b ros2-devel
RUN git clone https://github.com/RoverRobotics-forks/serial-ros2.git -b master
RUN git clone https://github.com/ros-drivers/joystick_drivers.git -b ros2
RUN git clone https://github.com/ros-tooling/topic_tools.git -b main
RUN git clone https://github.com/ros-perception/image_transport_plugins.git -b humble
RUN git clone git@github.com:frc-88/zed-ros2-wrapper.git -b tj2_detections
RUN git clone https://github.com/rst-tu-dortmund/costmap_converter -b ros2
RUN git clone https://github.com/rst-tu-dortmund/teb_local_planner.git -b ros2-master

WORKDIR ${BW_ROS_ROOT}

RUN colcon build
