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

${BASE_DIR}/upload.sh ${DESTINATION_NAME} ${REMOTE_KEY} n

SSH_COMMAND="ssh -i ${REMOTE_KEY} ${USERNAME}@${DESTINATION_NAME}"

# flash firmware
${SSH_COMMAND} -t "cd ${DEST_FULL_PATH}/firmware/bw_bcause && ./upload.sh"

${SSH_COMMAND} -t "${DEST_FULL_PATH}/firmware/bw_bcause/monitor.sh"

${BASE_DIR}/restart.sh ${DESTINATION_NAME} ${REMOTE_KEY} ${RESTART_ROSLAUNCH}
