#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y -r || true
