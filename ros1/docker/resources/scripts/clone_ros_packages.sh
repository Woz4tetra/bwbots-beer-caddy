#!/usr/bin/env bash
BASE_DIR=$(realpath "$(dirname $0)")

packages=(
    https://github.com/wjwwood/serial
    https://github.com/ros-drivers/joystick_drivers.git
    https://github.com/ros/geometry.git
    https://github.com/ros/geometry2.git
    https://github.com/frc-88/navigation.git
    https://github.com/ros-perception/vision_opencv.git
    https://github.com/ros-perception/vision_msgs.git
    https://github.com/frc-88/navigation_msgs.git
    https://github.com/magazino/move_base_flex.git
    https://github.com/introlab/find-object.git
    https://github.com/pal-robotics/ddynamic_reconfigure.git
    https://github.com/ros-perception/perception_pcl.git
    https://github.com/OctoMap/octomap_msgs.git
    https://github.com/ros-perception/pcl_msgs.git
    https://github.com/AprilRobotics/apriltag_ros.git
    https://github.com/rst-tu-dortmund/costmap_converter.git
    https://github.com/ros-perception/image_pipeline.git
    https://github.com/ros-perception/image_common.git
    https://github.com/robopeak/rplidar_ros.git
    https://github.com/ros-perception/laser_filters.git
    https://github.com/cra-ros-pkg/robot_localization
    https://github.com/ros-geographic-info/geographic_info.git
    https://github.com/ros-geographic-info/unique_identifier.git
    https://github.com/ros-perception/slam_gmapping.git
    https://github.com/ros-perception/openslam_gmapping.git
    https://github.com/ros-drivers/usb_cam.git
    https://github.com/ros-perception/image_transport_plugins
    https://github.com/ros/rosbag_snapshot.git
    https://github.com/frc-88/zed-ros-wrapper.git
    https://github.com/alireza-hosseini/ipcamera_driver
    https://github.com/ros-teleop/twist_mux.git
    https://github.com/ros-teleop/twist_mux_msgs.git
    https://github.com/iralabdisco/ira_laser_tools.git
    https://github.com/ros-perception/pointcloud_to_laserscan.git
    https://github.com/CCNYRoboticsLab/imu_tools.git
    https://github.com/ros/roslint.git
    https://github.com/ros/angles.git
    https://github.com/ros-perception/laser_geometry.git
    https://github.com/ros/diagnostics.git
    https://github.com/ros/robot_state_publisher.git
    https://github.com/ros/filters.git
    https://github.com/ros/urdf.git
    https://github.com/ros/kdl_parser.git
    https://github.com/ros/xacro.git
    https://github.com/ros/urdf_parser_py.git
    https://github.com/ros/xacro.git
    https://github.com/ros/executive_smach.git
    https://github.com/ros/joint_state_publisher.git
    https://github.com/splintered-reality/py_trees_ros.git
    https://github.com/splintered-reality/py_trees_msgs.git
    https://github.com/rst-tu-dortmund/teb_local_planner
    https://github.com/ros-visualization/interactive_markers.git
)

branches=(
    main      # https://github.com/wjwwood/serial
    main    # https://github.com/ros-drivers/joystick_drivers.git
    noetic-devel    # https://github.com/ros/geometry.git
    noetic-devel    # https://github.com/ros/geometry2.git
    noetic-devel    # https://github.com/frc-88/navigation.git
    noetic      # https://github.com/ros-perception/vision_opencv.git
    noetic-devel    # https://github.com/ros-perception/vision_msgs.git
    ros1    # https://github.com/frc-88/navigation_msgs.git
    noetic-devel    # https://github.com/magazino/move_base_flex.git
    noetic-devel    # https://github.com/introlab/find-object.git
    kinetic-devel   # https://github.com/pal-robotics/ddynamic_reconfigure.git
    melodic-devel   # https://github.com/ros-perception/perception_pcl.git
    melodic-devel   # https://github.com/OctoMap/octomap_msgs.git
    noetic-devel    # https://github.com/ros-perception/pcl_msgs.git
    master      # https://github.com/AprilRobotics/apriltag_ros.git
    master      # https://github.com/rst-tu-dortmund/costmap_converter.git
    noetic      # https://github.com/ros-perception/image_pipeline.git
    noetic-devel    # https://github.com/ros-perception/image_common.git
    master      # https://github.com/robopeak/rplidar_ros.git
    kinetic-devel   # https://github.com/ros-perception/laser_filters.git
    noetic-devel    # https://github.com/cra-ros-pkg/robot_localization
    master      # https://github.com/ros-geographic-info/geographic_info.git
    master      # https://github.com/ros-geographic-info/unique_identifier.git
    melodic-devel   # https://github.com/ros-perception/slam_gmapping.git
    melodic-devel   # https://github.com/ros-perception/openslam_gmapping.git
    develop     # https://github.com/ros-drivers/usb_cam.git
    noetic-devel    # https://github.com/ros-perception/image_transport_plugins
    main  # https://github.com/ros/rosbag_snapshot.git
    tj2_detections  # https://github.com/frc-88/zed-ros-wrapper.git
    master  # https://github.com/alireza-hosseini/ipcamera_driver
    melodic-devel  # https://github.com/ros-teleop/twist_mux.git
    melodic-devel  # https://github.com/ros-teleop/twist_mux_msgs.git
    ros1-master  # https://github.com/iralabdisco/ira_laser_tools.git
    lunar-devel  # https://github.com/ros-perception/pointcloud_to_laserscan.git
    noetic  # https://github.com/CCNYRoboticsLab/imu_tools.git
    master  # https://github.com/ros/roslint
    master  # https://github.com/ros/angles.git
    kinetic-devel  # https://github.com/ros-perception/laser_geometry.git
    noetic-devel  # https://github.com/ros/diagnostics.git
    noetic-devel  # https://github.com/ros/robot_state_publisher.git
    noetic-devel  # https://github.com/ros/filters.git
    melodic-devel  # https://github.com/ros/urdf.git
    noetic-devel  # https://github.com/ros/kdl_parser.git
    noetic-devel  # https://github.com/ros/xacro.git
    melodic-devel  # https://github.com/ros/urdf_parser_py.git
    noetic-devel  # https://github.com/ros/xacro.git
    noetic-devel  # https://github.com/ros/executive_smach.git
    noetic-devel  # https://github.com/ros/joint_state_publisher.git
    release/0.6.x  # https://github.com/splintered-reality/py_trees_ros.git
    release/0.3.x  # https://github.com/splintered-reality/py_trees_msgs.git
    noetic-devel    # https://github.com/rst-tu-dortmund/teb_local_planner
    noetic-devel    # https://github.com/ros-visualization/interactive_markers.git
)

len=${#packages[@]}
for (( i=0; i<$len; i++ )); do
    git clone --recursive ${packages[i]} --branch ${branches[i]}
done
