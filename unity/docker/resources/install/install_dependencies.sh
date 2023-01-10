BASE_DIR=$(realpath "$(dirname $0)")

echo "---"
echo "Installing dependencies via git and catkin_make"
echo "---"

repos=(
    https://github.com/robopeak/rplidar_ros.git
    https://github.com/wjwwood/serial
)

branches=(
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

catkin_make -DCATKIN_BLACKLIST_PACKAGES="bw_yolo;bw_zed;zed_nodelets;zed_ros;zed_wrapper"

success=$?
if [[ $success -eq 0 ]];
then
    echo "Packages built successfully"
else
    echo "Something went wrong!"
    exit 1
fi
