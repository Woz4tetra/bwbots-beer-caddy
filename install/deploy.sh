BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(dirname $BASE_DIR)
DESTINATION_NAME=$1
REMOTE_KEY=$2
RESTART_ROSLAUNCH=$3

USERNAME=nvidia
LOCAL_PATH=${PARENT_DIR}
DESTINATION_PATH=/home/${USERNAME}

if [ -z ${DESTINATION_NAME} ]; then
    echo "Please set a destination IP or hostname"
    exit
fi

if [ -z ${REMOTE_KEY} ]; then
    echo "Please set an SSH key file"
    exit
fi

LOCAL_PATH=$(realpath $LOCAL_PATH)
LOCAL_NAME=$(basename $LOCAL_PATH)
DEST_FULL_PATH=${DESTINATION_PATH}/${LOCAL_NAME}
CATKIN_WS_PATH=${DESTINATION_PATH}/ros_ws
PACKAGES_PATH=${LOCAL_PATH}/ros1/bwbots

${BASE_DIR}/upload.sh ${DESTINATION_NAME} ${REMOTE_KEY} n

SSH_COMMAND="ssh -i ${REMOTE_KEY} ${USERNAME}@${DESTINATION_NAME}"

# stop roslaunch
echo "Stopping roslaunch"
${SSH_COMMAND} -t "sudo systemctl stop roslaunch.service"

# build db_tools
${SSH_COMMAND} "bash ${DEST_FULL_PATH}/ros1/install/robot_installation/09_install_python_libraries.sh"

# build catkin ws

PACKAGE_LIST=`ls $PACKAGES_PATH`
PACKAGE_LIST=`echo "$PACKAGE_LIST" | tr '\n' ';'`

${SSH_COMMAND} -t "export OPENBLAS_CORETYPE=ARMV8 && cd ${CATKIN_WS_PATH} && source /home/${USERNAME}/noetic_ws/install_isolated/setup.bash && source ${CATKIN_WS_PATH}/devel/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES='$PACKAGE_LIST'"

${BASE_DIR}/restart.sh ${DESTINATION_NAME} ${REMOTE_KEY} ${RESTART_ROSLAUNCH}
