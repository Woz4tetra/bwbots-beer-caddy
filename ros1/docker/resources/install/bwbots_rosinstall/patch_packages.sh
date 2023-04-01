#!/usr/bin/env bash
set -e

find ${DEP_ROS_WS_SRC} -type f -name CMakeLists.txt -exec sed -i'' -e 's/Boost REQUIRED python37/Boost REQUIRED python3/g' {} +

# create a patch file: https://stackoverflow.com/questions/6658313/how-can-i-generate-a-git-patch-for-a-specific-commit
# helpful forum post: https://stackoverflow.com/questions/4770177/git-patch-does-not-apply
cd ${DEP_ROS_WS_SRC}/image_pipeline/
git checkout -f
git apply /root/install/bwbots_rosinstall/fix-image-pipeline.patch --reject --whitespace=fix 

cd ${DEP_ROS_WS_SRC}/geometry2/
git checkout -f
git apply /root/install/bwbots_rosinstall/fix-geometry2.patch --reject --whitespace=fix 

cd ${DEP_ROS_WS_SRC}/imu_tools/
git checkout -f
git apply /root/install/bwbots_rosinstall/fix-imu-tools.patch --reject --whitespace=fix 

cd ${DEP_ROS_WS_SRC}/zed-ros-wrapper/
touch rviz-plugin-zed-od/CATKIN_IGNORE
