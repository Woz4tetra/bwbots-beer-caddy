BASE_DIR=$(realpath "$(dirname $0)")
LOCAL_PATH=${BASE_DIR}/..
LOCAL_PATH=$(realpath $LOCAL_PATH)
PACKAGES_PATH=${LOCAL_PATH}/bwbots

PACKAGE_LIST=`ls $PACKAGES_PATH`
PACKAGE_LIST=`echo "$PACKAGE_LIST" | grep 'bw_' | tr '\n' ';'`
cd ~/ros_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES=$PACKAGE_LIST