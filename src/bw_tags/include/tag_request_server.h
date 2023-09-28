#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "bw_tags/RequestTags.h"

class TagRequestServer
{
public:
    int run();
    TagRequestServer(ros::NodeHandle* nodehandle);
private:
    ros::NodeHandle nh;

    ros::Duration _timeout;
    ros::Duration _in_sync_threshold;

    bool _first_sent;

    ros::Publisher _info_out_pub;
    ros::Publisher _image_out_pub;

    ros::Subscriber _info_in_sub;
    ros::Subscriber _image_in_sub;
    ros::Subscriber _tags_sub;

    sensor_msgs::ImagePtr _stored_image;
    sensor_msgs::CameraInfoPtr _stored_info;
    apriltag_ros::AprilTagDetectionArrayPtr _stored_tags;

    ros::ServiceServer _tags_request_srv;

    void image_callback(const sensor_msgs::ImageConstPtr& color_image);
    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info);
    void tag_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& tags);
    bool tag_request_callback(bw_tags::RequestTags::Request &req, bw_tags::RequestTags::Response &resp);

    bool wait_for_image();
};
