DESTINATION_NAME=$1
REMOTE_KEY=$2
RESTART_SERVICE=$3
USERNAME=nvidia
SERVICE_NAME=bwbots

if [ -z ${DESTINATION_NAME} ]; then
    echo "Please set a destination IP or hostname"
    exit
fi

if [ -z ${REMOTE_KEY} ]; then
    echo "Please set an SSH key file"
    exit
fi

SSH_COMMAND="ssh -i ${REMOTE_KEY} ${USERNAME}@${DESTINATION_NAME}"

# restart systemd
if [ -z $RESTART_SERVICE ]; then
    echo "Restart ${SERVICE_NAME}.service? (Y/n) "
    read response
    case $response in
      ([Nn])     echo "Skipping restart";;
      (*)        echo "Restarting ${SERVICE_NAME}." && ${SSH_COMMAND} -t "sudo systemctl restart ${SERVICE_NAME}.service";;
    esac
else
    if [[ $RESTART_SERVICE == "n" ]]; then
        echo "Skipping restart"
    else
        echo "Restarting ${SERVICE_NAME}." && ${SSH_COMMAND} -t "sudo systemctl restart ${SERVICE_NAME}.service"
    fi
fi
