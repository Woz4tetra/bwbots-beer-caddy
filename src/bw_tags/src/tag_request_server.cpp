#include "tag_request_server.h"


TagRequestServer::TagRequestServer(ros::NodeHandle* nodehandle) : nh(*nodehandle)
{
    double timeout_s;
    ros::param::param<double>("~timeout", timeout_s, 1.0);
    _timeout = ros::Duration(timeout_s);

    _image_out_pub = nh.advertise<sensor_msgs::Image>("out/image_raw", 1);
    _info_out_pub = nh.advertise<sensor_msgs::CameraInfo>("out/camera_info", 1);

    _image_in_sub = nh.subscribe<sensor_msgs::Image>("in/image_raw", 1, &TagRequestServer::image_callback, this);
    _info_in_sub = nh.subscribe<sensor_msgs::CameraInfo>("in/camera_info", 1, &TagRequestServer::camera_info_callback, this);
    _tags_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("tag_detections", 1, &TagRequestServer::tag_callback, this);

    _tags_request_srv = nh.advertiseService("tags_request", &TagRequestServer::tag_request_callback, this);

    _stored_image.reset(new sensor_msgs::Image);
    _stored_info.reset(new sensor_msgs::CameraInfo);
    _stored_tags.reset(new apriltag_ros::AprilTagDetectionArray);

    _got_image = false;
    _first_sent = false;
}

void TagRequestServer::image_callback(const sensor_msgs::ImageConstPtr& color_image)
{
    _stored_image = boost::const_pointer_cast<sensor_msgs::Image>(color_image);
    _got_image = true;
}

void TagRequestServer::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    _stored_info = boost::const_pointer_cast<sensor_msgs::CameraInfo>(camera_info);
    if (_got_image && !_first_sent) {
        _first_sent = true;
        _image_out_pub.publish(*_stored_image);
        _info_out_pub.publish(*_stored_info);
    }
}

void TagRequestServer::tag_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& tags)
{
    _stored_tags = boost::const_pointer_cast<apriltag_ros::AprilTagDetectionArray>(tags);
    ROS_INFO("Got tag: %f", _stored_tags->header.stamp.toSec());
}

bool TagRequestServer::tag_request_callback(bw_tags::RequestTags::Request &req, bw_tags::RequestTags::Response &resp)
{
    ros::Time start_time = ros::Time::now();
    if (!_stored_image || !_stored_info) {
        ROS_ERROR("Image isn't set. Can't fulfill tag request");
        return false;
    }
    
    if (!wait_for_image()) {
        ROS_ERROR("Timed out waiting for image");
        return false;
    }

    ros::Time prev_stamp = _stored_tags->header.stamp;
    _image_out_pub.publish(*_stored_image);
    _info_out_pub.publish(*_stored_info);
    bool success = false;
    while (ros::Time::now() - start_time < _timeout) {
        ros::spinOnce();
        if (prev_stamp < _stored_tags->header.stamp) {
            resp.tags = *_stored_tags;
            success = true;
            break;
        }
    }
    if (!success) {
        resp.tags = apriltag_ros::AprilTagDetectionArray();
        ROS_ERROR("Failed to get tag detections within the allotted time!");
    }
    return true;
}

bool TagRequestServer::wait_for_image() {
    ros::Time start_time = ros::Time::now();
    while (ros::Time::now() - start_time < _timeout) {
        if (_stored_info->header.stamp >= start_time) {
            return true;
        }
        ros::spinOnce();
    }
    return false;
}

int TagRequestServer::run()
{
    ros::spin();
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_request_server");
    ros::NodeHandle nh;
    TagRequestServer node(&nh);
    return node.run();
}
