#!/bin/bash
echo "Starting bwbots"

SESSION=bwbots

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d
fi

tmux send -t $SESSION "source ${HOME}/install/enable_tmux_logger.sh bw" ENTER
tmux send -t $SESSION "source /opt/ros/${ROS_DISTRO}/setup.bash" ENTER
tmux send -t $SESSION "source ${HOME}/ros_ws/devel/setup.bash" ENTER
tmux send -t $SESSION "source ${HOME}/scripts/startup.sh" ENTER
tmux send -t $SESSION "roslaunch --wait bw_bringup bw_bringup.launch --screen" ENTER

sleep 2
${HOME}/scripts/tail-session.sh
sleep infinity
