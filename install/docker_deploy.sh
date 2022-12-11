BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(dirname $BASE_DIR)
DESTINATION_NAME=$1
REMOTE_KEY=$2
RESTART_SERVICE=$3
SERVICE_NAME=bwbots

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

# stop service
echo "Stopping ${SERVICE_NAME}"
${SSH_COMMAND} -t "sudo systemctl stop ${SERVICE_NAME}.service"

# build bwbots
${SSH_COMMAND} "sudo ${DEST_FULL_PATH}/ros1/docker/native/build_container.sh"
${SSH_COMMAND} "${DEST_FULL_PATH}/ros1/docker/native/post_build.sh"

${BASE_DIR}/restart.sh ${DESTINATION_NAME} ${REMOTE_KEY} ${RESTART_SERVICE}
