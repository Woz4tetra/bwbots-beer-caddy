BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(dirname $BASE_DIR)
DESTINATION_NAME=$1
REMOTE_KEY=$2
RESTART_ROSLAUNCH=$3

USERNAME=nvidia
LOCAL_PATH=${PARENT_DIR}
DESTINATION_PATH=/home/${USERNAME}
LOCAL_NAME=$(basename $LOCAL_PATH)
DEST_FULL_PATH=${DESTINATION_PATH}/${LOCAL_NAME}

if [ -z ${DESTINATION_NAME} ]; then
    echo "Please set a destination IP or hostname"
    exit
fi

if [ -z ${REMOTE_KEY} ]; then
    echo "Please set an SSH key file"
    exit
fi

SSH_COMMAND="ssh -i ${REMOTE_KEY} ${USERNAME}@${DESTINATION_NAME}"

OUTPUT=$( rsync -avur --exclude-from=${LOCAL_PATH}/install/exclude.txt  -e "ssh -i ${REMOTE_KEY}"  ${LOCAL_PATH} ${USERNAME}@${DESTINATION_NAME}:${DESTINATION_PATH} | tee /dev/tty)

if echo "$OUTPUT" | grep -q 'bwbots-beer-caddy/ros1/bwbots/bw_tools/'; then
    # build bw_tools
    ${SSH_COMMAND} "bash ${DEST_FULL_PATH}/ros1/install/robot_installation/09_install_python_libraries.sh"
fi

${BASE_DIR}/restart.sh ${DESTINATION_NAME} ${REMOTE_KEY} ${RESTART_ROSLAUNCH}
