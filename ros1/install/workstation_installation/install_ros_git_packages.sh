BASE_DIR=$(realpath "$(dirname $0)")

ROS_WS=$HOME/ros_ws
ROS_WS_SRC=${ROS_WS}/src

echo "---"
echo "Installing dependencies via git and catkin_make"
echo "---"

repos=(
    git@github.com:frc-88/navigation.git
    git@github.com:frc-88/navigation_msgs.git
    git@github.com:frc-88/zed-ros-wrapper.git
    https://github.com/robopeak/rplidar_ros.git
    https://github.com/wjwwood/serial
)

branches=(
    noetic-devel    # git@github.com:frc-88/navigation.git
    ros1    # git@github.com:frc-88/navigation_msgs.git
    tj2_detections  # git@github.com:frc-88/zed-ros-wrapper.git
    master      # https://github.com/robopeak/rplidar_ros.git
    main      # https://github.com/wjwwood/serial
)

mkdir -p ${ROS_WS_SRC}
cd ${ROS_WS_SRC}

len=${#repos[@]}
for (( i=0; i<$len; i++ )); do
    git clone --recursive ${repos[i]} --branch ${branches[i]}
done

echo "Building dependencies"

cd ${ROS_WS}

rosdep install --from-paths src --ignore-src --rosdistro=noetic -y -r

if sudo lshw -C display | grep -i nvidia; then
    echo "\n\nNVidia GPU detected. Assuming cuda libraries are installed\n\n"
    /opt/ros/noetic/bin/catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
else 
    echo "\n\nNVidia GPU not detected. Skipping cuda packages\n\n"
    /opt/ros/noetic/bin/catkin_make -DCATKIN_BLACKLIST_PACKAGES="bw_yolo;zed_nodelets;zed_ros;zed_wrapper" -DPYTHON_EXECUTABLE=/usr/bin/python3
fi


success=$?
if [[ $success -eq 0 ]];
then
    echo "Packages built successfully"
else
    echo "Something went wrong!"
    exit 1
fi
