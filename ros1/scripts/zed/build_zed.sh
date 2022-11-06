BASE_DIR=$(realpath "$(dirname $0)")

CATKIN_WS_PATH=~/ros_ws
PACKAGES_PATH=~/bwbots-beer-caddy/ros1/bwbots
USERNAME=nvidia

cd ${CATKIN_WS_PATH}/src/zed-ros-wrapper
git pull

PACKAGE_LIST=`ls -d */ | sed 's/\///g'`
PACKAGE_LIST=`echo "$PACKAGE_LIST" | tr '\n' ';'`

cd ${CATKIN_WS_PATH}

export OPENBLAS_CORETYPE=ARMV8
source /home/${USERNAME}/noetic_ws/install_isolated/setup.bash
source ${CATKIN_WS_PATH}/devel/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="$PACKAGE_LIST"
