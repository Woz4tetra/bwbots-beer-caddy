#pragma once

#include <string>
#include <iostream>
#include <boost/format.hpp>

#include "ros/ros.h"
#include "ros/console.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>

#include <std_msgs/ColorRGBA.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

#include <dynamic_reconfigure/server.h>
#include <bw_yolo/YoloDetectionConfig.h>

#include "detector.h"


geometry_msgs::Point make_point(double x, double y, double z) {
    geometry_msgs::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    return pt;
}


class BwYolo
{
public:
    BwYolo(ros::NodeHandle* nodehandle);
    int run();
private:
    ros::NodeHandle nh;  // ROS node handle

    // Parameters
    std::string _model_path;
    std::string _classes_path;
    float _conf_threshold;
    float _iou_threshold;
    std::string _image_width_param;
    std::string _image_height_param;
    int _image_width;
    int _image_height;
    double _min_depth, _max_depth;
    bool _publish_overlay;
    bool _report_loop_times;
    std::string _target_frame;
    double _marker_persistance_s;
    double _message_delay_warning_ms, _long_loop_warning_ms;
    std::map<std::string, double> _z_depth_estimations;
    int _relative_threshold;
    double _depth_num_std_devs, _empty_mask_num_std_devs;
    double _cube_opacity, _visuals_line_width;
    int _erosion_size;

    // Members
    Detector* _detector;
    std::vector<std::string> _class_names;
    image_geometry::PinholeCameraModel _camera_model;
    std::vector<int> _obj_count;
    ros::Duration _marker_persistance;
    sensor_msgs::CameraInfo _camera_info;
    cv::Mat erode_element;

    // Subscribers
    ros::Subscriber _color_info_sub;
    message_filters::Subscriber<sensor_msgs::Image> _color_sub;
    message_filters::Subscriber<sensor_msgs::Image> _depth_sub;

    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproxSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactSyncPolicy;

    // typedef message_filters::Synchronizer<ApproxSyncPolicy> Sync;
    typedef message_filters::Synchronizer<ExactSyncPolicy> Sync;
    boost::shared_ptr<Sync> _sync;

    // Publishers
    image_transport::ImageTransport _image_transport;
    ros::Publisher _detection_pub;
    ros::Publisher _marker_pub;
    image_transport::Publisher _overlay_pub;
    ros::Publisher _overlay_info_pub;

    // Dynamic reconfigure
    dynamic_reconfigure::Server<bw_yolo::YoloDetectionConfig> _dyn_cfg;
    dynamic_reconfigure::Server<bw_yolo::YoloDetectionConfig>::CallbackType _dyn_cfg_wrapped_callback;
    void dynamic_callback(bw_yolo::YoloDetectionConfig &config, uint32_t level);

    // ROS TF
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    // Callbacks
    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info);
    void rgbd_callback(const sensor_msgs::ImageConstPtr& color_image, const sensor_msgs::ImageConstPtr& depth_image);

    // Helpers
    std_msgs::ColorRGBA get_detection_color(cv::Mat color_cv_image, cv::Mat mask);
    void get_depth_from_detection(cv::Mat depth_cv_image, vision_msgs::Detection2D detection_2d_msg, cv::Mat& out_mask, double& z_min, double& z_max);
    vision_msgs::Detection3D detection_2d_to_3d(vision_msgs::Detection2D detection_2d_msg, double z_min, double z_max);
    void tf_detection_pose_to_robot(vision_msgs::Detection3D& detection_3d_msg);
    std::string get_class_name(int obj_id);
    int get_class_index(int obj_id);
    int get_class_count(int obj_id);
    vision_msgs::Detection2DArray detections_to_msg(const std::vector<std::vector<Detection>>& detections);
    void draw_overlay(cv::Mat img, const std::vector<std::vector<Detection>>& detections, cv::Mat debug_mask, bool label = true);
    void add_detection_to_marker_array(visualization_msgs::MarkerArray& marker_array, vision_msgs::Detection3D detection_3d_msg, std_msgs::ColorRGBA color);
    visualization_msgs::Marker make_marker(vision_msgs::Detection3D detection_3d_msg, std_msgs::ColorRGBA color);
    double get_depth_conversion(std::string encoding);

};

