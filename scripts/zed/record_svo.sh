#!/bin/bash
mkdir -p /media/storage/svo

int_handler() {
    echo "Caught stop signal"
    rosservice call /zed/stop_svo_recording
    exit 0;
}
trap 'int_handler' SIGINT
rosservice call /zed/start_svo_recording "{svo_filename: /media/storage/svo/zed_$(date +"%Y-%m-%dT%H-%M-%S").svo}"
sleep infinity
