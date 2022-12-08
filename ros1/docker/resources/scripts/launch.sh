
#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${HOME}/ros_ws/install/setup.bash
source ${HOME}/bw_ros_ws/install/setup.bash

roslaunch bw_bringup bw_bringup.launch --screen
