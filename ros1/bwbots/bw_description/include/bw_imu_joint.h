#pragma once

#include "ros/ros.h"

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;


class BwImuJoint
{
public:
    BwImuJoint(ros::NodeHandle* nodehandle);
    int run();
private:
    ros::NodeHandle nh;  // ROS node handle

    // Parameters
    double _publish_rate;
    bool _navx_as_base;
    string _base_parent_frame;
    string _base_child_frame;
    string _base_imu_frame;

    // Members
    geometry_msgs::Quaternion base_quat_msg;
    tf2::Quaternion _static_imu_to_base_quat, _base_to_base_tilt_quat;
    tf2::Matrix3x3 _static_imu_to_base_mat;
    geometry_msgs::TransformStamped _static_imu_to_base_tf;
    bool _static_imu_tf_set;

    // Publishers
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    
    // Subscribers
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    ros::Subscriber _base_imu_sub;

    // Sub callbacks
    void base_imu_callback(const sensor_msgs::ImuConstPtr& imu);

    void base_callback(tf2::Quaternion quat);
    void publish_base_tf();
};
