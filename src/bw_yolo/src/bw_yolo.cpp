#include "bw_yolo.h"

BwYolo::BwYolo(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _image_transport(nh),
    tfListener(tfBuffer)
{
    ros::param::param<std::string>("~model_path", _model_path, "best.pt");
    ros::param::param<std::string>("~classes_path", _classes_path, "coco.names");
    ros::param::param<std::string>("~model_path", _model_path, "best.pt");
    ros::param::param<float>("~confidence_threshold", _conf_threshold, 0.25);
    ros::param::param<float>("~nms_iou_threshold", _iou_threshold, 0.45);

    ros::param::param<int>("~warmup_image_width", _image_width, 960);
    ros::param::param<int>("~warmup_image_height", _image_height, 540);

    ros::param::param<int>("~relative_threshold", _relative_threshold, 128);

    ros::param::param<double>("~min_depth", _min_depth, 0.1);
    ros::param::param<double>("~max_depth", _max_depth, 10.0);
    ros::param::param<double>("~depth_num_std_devs", _depth_num_std_devs, 1.0);
    
    double message_delay_warning;
    ros::param::param<double>("~message_delay_warning", message_delay_warning, 0.5);
    _message_delay_warning_ms = message_delay_warning * 1000.0;

    double long_loop_warning;
    ros::param::param<double>("~long_loop_warning_ms", long_loop_warning, 0.075);
    _long_loop_warning_ms = long_loop_warning * 1000.0;

    ros::param::param<double>("~visuals_cube_opacity", _cube_opacity, 0.75);
    ros::param::param<double>("~visuals_line_width", _visuals_line_width, 0.01);
    ros::param::param<int>("~erosion_size", _erosion_size, 3);

    ros::param::param<bool>("~publish_overlay", _publish_overlay, false);
    ros::param::param<bool>("~report_loop_times", _report_loop_times, true);
    ros::param::param<std::string>("~target_frame", _target_frame, "");

    double marker_persistance_s;
    ros::param::param<double>("~marker_persistance", marker_persistance_s, 0.0);
    _marker_persistance = ros::Duration(marker_persistance_s);

    double image_sync_threshold_s;
    ros::param::param<double>("~image_sync_threshold", image_sync_threshold_s, 0.01);
    _image_sync_threshold = ros::Duration(image_sync_threshold_s);

    int starting_mode;
    ros::param::param<int>("~starting_mode", starting_mode, APPROX_CONTINUOUS_MODE);
    ros::param::param<int>("~queue_size", _queue_size, 1);

    torch::DeviceType device_type;
    if (torch::cuda::is_available()) {
        device_type = torch::kCUDA;
    } else {
        device_type = torch::kCPU;
        ROS_WARN("Using CPU to run inference!");
    }
    _class_names = Detector::LoadNames(_classes_path);
    if (_class_names.empty()) {
        std::cerr << "Error loading class names!\n";
        std::exit(EXIT_FAILURE);
    }
    _obj_count.resize(_class_names.size());

    erode_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * _erosion_size + 1, 2 * _erosion_size + 1));

    _box_point_permutations = {
        {-1, -1,  1},
        { 1,  1,  1},
        { 1, -1,  1},
        {-1, -1,  1},
        {-1, -1, -1},
        { 1,  1, -1},
        { 1, -1, -1},
        {-1, -1, -1},
    };

    _detector = new Detector(_model_path, device_type, _report_loop_times);
    // run twice to warm up
    ROS_INFO("Warming up detector with (%dx%d)", _image_width, _image_height);
    cv::Mat warmup_img = cv::Mat::zeros(_image_height, _image_width, CV_8UC3);
    cv::randu(warmup_img, cv::Scalar(0), cv::Scalar(255));
    for (size_t count = 0; count < 2; count++) {
        _detector->Run(warmup_img, _conf_threshold, _iou_threshold);        
    }
    _color_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("color/camera_info", 1, &BwYolo::camera_info_callback, this);

    _detection_pub = nh.advertise<bw_yolo::DetectionObjectsStamped>("detections", 25);
    _marker_pub = nh.advertise<visualization_msgs::MarkerArray>("detections/markers", 25);

    _overlay_pub = _image_transport.advertise("overlay/image_raw", 2);
    _overlay_info_pub = nh.advertise<sensor_msgs::CameraInfo>("overlay/camera_info", 2);
    _frame_request_srv = nh.advertiseService("detection_request", &BwYolo::frame_request_callback, this);

    set_mode(starting_mode);

    _dyn_cfg_wrapped_callback = boost::bind(&BwYolo::dynamic_callback, this, _1, _2);
    _dyn_cfg.setCallback(_dyn_cfg_wrapped_callback);

    ROS_INFO("bw_yolo is ready");
}

bool BwYolo::set_mode(int mode) {
    ROS_INFO("Setting mode to %d", mode);
    _mode = mode;

    switch (mode)
    {
    case APPROX_CONTINUOUS_MODE:
        _color_sync_sub.subscribe(nh, "color/image_raw", _queue_size);
        _depth_sync_sub.subscribe(nh, "depth/image_raw", _queue_size);
        _approx_sync.reset(new ApproxSync(ApproxSyncPolicy(_queue_size), _color_sync_sub, _depth_sync_sub));
        _approx_sync->registerCallback(boost::bind(&BwYolo::rgbd_callback, this, _1, _2));
        break;
    case EXACT_CONTINUOUS_MODE:
        _color_sync_sub.subscribe(nh, "color/image_raw", _queue_size);
        _depth_sync_sub.subscribe(nh, "depth/image_raw", _queue_size);
        _exact_sync.reset(new ExactSync(ExactSyncPolicy(_queue_size), _color_sync_sub, _depth_sync_sub));
        _exact_sync->registerCallback(boost::bind(&BwYolo::rgbd_callback, this, _1, _2));
        break;
    case REQUEST_MODE:
        _color_sub = nh.subscribe<sensor_msgs::Image>("color/image_raw", _queue_size, &BwYolo::rgb_callback, this);
        _depth_sub = nh.subscribe<sensor_msgs::Image>("depth/image_raw", _queue_size, &BwYolo::depth_callback, this);
        break;
    default:
        return false;
    }
    return true;
}

void BwYolo::dynamic_callback(bw_yolo::YoloDetectionConfig &config, uint32_t level)
{
    _conf_threshold = config.confidence_threshold;
    _iou_threshold = config.nms_iou_threshold;
    _publish_overlay = config.publish_overlay;
    _report_loop_times = config.report_loop_times;

    ROS_INFO("Setting config to:\n\tconfidence_threshold: %0.2f\n\tnms_iou_threshold: %0.2f\n\tpublish_overlay: %d\n\treport_loop_times: %d",
        _conf_threshold,
        _iou_threshold,
        _publish_overlay,
        _report_loop_times
    );
}

void BwYolo::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    _camera_info = *camera_info;
    _camera_model.fromCameraInfo(camera_info);
    _color_info_sub.shutdown();
    ROS_INFO("Camera model loaded");
}

double BwYolo::get_depth_conversion(std::string encoding)
{
  switch (sensor_msgs::image_encodings::bitDepth(encoding)) {
    case 8:
    case 16:
      return 0.001;
    case 32:
    case 64:
    default:
      return 1.0;
  };
}

void BwYolo::rgb_callback(const sensor_msgs::ImageConstPtr& color_image)
{
    set_color_image(color_image);
}

void BwYolo::depth_callback(const sensor_msgs::ImageConstPtr& depth_image)
{
    set_depth_image(depth_image);
}

void BwYolo::rgbd_callback(const sensor_msgs::ImageConstPtr& color_image, const sensor_msgs::ImageConstPtr& depth_image)
{
    if (set_color_image(color_image) && set_depth_image(depth_image)) {
        _detection_result = detection_pipeline(_color_header, _depth_header, _depth_encoding, _color_cv_image, _depth_cv_image);
    }
    _detection_pub.publish(_detection_result);
}
bool BwYolo::frame_request_callback(bw_yolo::RequestFrame::Request &req, bw_yolo::RequestFrame::Response &resp) {
    if (_mode == REQUEST_MODE) {
        _detection_result = detection_pipeline(_color_header, _depth_header, _depth_encoding, _color_cv_image, _depth_cv_image);
    }
    resp.detections = _detection_result;
    return true;
}

bool BwYolo::set_color_image(const sensor_msgs::ImageConstPtr& color_image)
{
    ros::Time now = ros::Time::now();
    ros::Duration color_transport_delay = now - color_image->header.stamp;
    double color_transport_delay_ms = color_transport_delay.toSec() * 1000.0;
    if (color_transport_delay_ms >= _message_delay_warning_ms) {
        ROS_WARN_THROTTLE(0.5, "Color image has a large delay: %0.4f ms", color_transport_delay_ms);
    }

    cv_bridge::CvImagePtr color_ptr;
    try {
        color_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to convert color image: %s", e.what());
        return false;
    }
    _color_cv_image = color_ptr->image;
    _color_header = color_image->header;
    _color_encoding = color_ptr->encoding;
    return true;

}

bool BwYolo::set_depth_image(const sensor_msgs::ImageConstPtr& depth_image)
{
    ros::Time now = ros::Time::now();
    ros::Duration depth_transport_delay = now - depth_image->header.stamp;
    double depth_transport_delay_ms = depth_transport_delay.toSec() * 1000.0;
    if (depth_transport_delay_ms >= _message_delay_warning_ms) {
        ROS_WARN_THROTTLE(0.5, "Depth image has a large delay: %0.4f ms", depth_transport_delay_ms);
    }
    cv_bridge::CvImagePtr depth_ptr;
    try {
        depth_ptr = cv_bridge::toCvCopy(depth_image);  // encoding: passthrough
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to convert depth image: %s", e.what());
        return false;
    }
    _depth_cv_image = depth_ptr->image;
    _depth_header = depth_ptr->header;
    _depth_encoding = depth_ptr->encoding;
    return true;
}

bw_yolo::DetectionObjectsStamped BwYolo::detection_pipeline(
    std_msgs::Header color_header,
    std_msgs::Header depth_header,
    std::string depth_encoding,
    cv::Mat color_cv_image,
    cv::Mat depth_cv_image)
{
    bw_yolo::DetectionObjectsStamped objects;

    if (color_header.stamp - depth_header.stamp > _image_sync_threshold) {
        double delay = (color_header.stamp - depth_header.stamp).toSec();
        ROS_ERROR("Color and depth images are out of sync: %f s", delay);
        return objects;
    }

    if (color_cv_image.cols == 0 || color_cv_image.rows == 0) {
        ROS_ERROR("Color image has a zero width dimension!");
        return objects;
    }
    if (depth_cv_image.cols == 0 || depth_cv_image.rows == 0) {
        ROS_ERROR("Depth image has a zero width dimension!");
        return objects;
    }

    auto t_start = std::chrono::high_resolution_clock::now();
    auto result = _detector->Run(color_cv_image, _conf_threshold, _iou_threshold);
    if (_report_loop_times) {
        ROS_INFO("----- %lu detections -----\n%s", 
            result.size(),
            _detector->GetTimingReport().c_str()
        );
    }
    vision_msgs::Detection3DArray detection_3d_arr_msg;
    detection_3d_arr_msg.header = color_header;

    if (result.empty()) {
        return objects;
    }
    auto t0 = std::chrono::high_resolution_clock::now();
    auto detection_time_s = std::chrono::duration<double>(t0 - t_start);
    if (detection_time_s.count() > 1.0) {
        ROS_INFO("Detector is warming up");
        return objects;
    }

    auto t1 = std::chrono::high_resolution_clock::now();

    vision_msgs::Detection2DArray detection_2d_arr_msg = detections_to_msg(result);
    detection_2d_arr_msg.header = color_header;

    visualization_msgs::MarkerArray marker_array;

    cv::patchNaNs(depth_cv_image, 0.0);
    double conversion = get_depth_conversion(depth_encoding);
    depth_cv_image *= conversion;

    cv::Mat debug_mask = cv::Mat::zeros(depth_cv_image.rows, depth_cv_image.cols, CV_8UC1);
    for (size_t index = 0; index < detection_2d_arr_msg.detections.size(); index++) {
        vision_msgs::Detection2D detection_2d_msg = detection_2d_arr_msg.detections[index];
        detection_2d_msg.header = color_header;
        
        double z_min, z_max;
        cv::Mat detection_mask = cv::Mat::zeros(depth_cv_image.rows, depth_cv_image.cols, CV_8UC1);
        get_depth_from_detection(depth_cv_image, detection_2d_msg, detection_mask, z_min, z_max);
        cv::bitwise_or(debug_mask, detection_mask, debug_mask);

        std_msgs::ColorRGBA obj_color = get_detection_color(color_cv_image, detection_mask);        
        vision_msgs::Detection3D detection_3d_msg = detection_2d_to_3d(detection_2d_msg, z_min, z_max);
        add_detection_to_marker_array(marker_array, detection_3d_msg, obj_color);

        tf_detection_pose_to_robot(detection_3d_msg);
        detection_3d_arr_msg.header.frame_id = detection_3d_msg.header.frame_id;
        detection_3d_arr_msg.detections.push_back(detection_3d_msg);
    }
    _marker_pub.publish(marker_array);
    auto t_end = std::chrono::high_resolution_clock::now();

    auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
    if ((double)total_time.count() >= _long_loop_warning_ms) {
        ROS_WARN("Detection loop took a long time: %ld ms", total_time.count());
    }

    if (_publish_overlay && _overlay_pub.getNumSubscribers() > 0)
    {
        cv::Mat overlay = color_cv_image.clone();
        draw_overlay(overlay, result, debug_mask);
        sensor_msgs::ImagePtr overlay_msg = cv_bridge::CvImage(color_header, sensor_msgs::image_encodings::BGR8, overlay).toImageMsg();

        _overlay_pub.publish(overlay_msg);
        _overlay_info_pub.publish(_camera_info);
    }
    if (_report_loop_times) {
        auto detection_time = std::chrono::duration_cast<std::chrono::milliseconds>(t0 - t_start);
        auto overlay_time = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
        auto message_prep_time = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t1);
        boost::format fmt = boost::format("\tdetection: %d ms\n\toverlay: %d ms\n\tmessage: %d ms\n\ttotal: %d ms\n") %
            detection_time.count() %
            overlay_time.count() %
            message_prep_time.count() %
            total_time.count();
        ROS_INFO("Loop report:\n%s", fmt.str().c_str());
    }

    objects = convert_to_objects(color_cv_image.cols, color_cv_image.rows, detection_2d_arr_msg, detection_3d_arr_msg);

    return objects;
}

std_msgs::ColorRGBA BwYolo::get_detection_color(cv::Mat color_cv_image, cv::Mat mask)
{
    cv::Scalar bgr_pixel = cv::mean(color_cv_image, mask);
    double b = bgr_pixel[0] / 255.0;
    double g = bgr_pixel[1] / 255.0;
    double r = bgr_pixel[2] / 255.0;
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1.0;
    return color;
}

void BwYolo::get_depth_from_detection(cv::Mat depth_cv_image, vision_msgs::Detection2D detection_2d_msg, cv::Mat& out_mask, double& z_min, double& z_max)
{
    // assumes depth_cv_image has been converted to CV_32FC1 where 1.0 == 1 meter
    // extract pixel coordinates of bounding box
    int center_x = (int)(detection_2d_msg.bbox.center.x);
    int center_y = (int)(detection_2d_msg.bbox.center.y);
    int size_x = (int)(detection_2d_msg.bbox.size_x);
    int size_y = (int)(detection_2d_msg.bbox.size_y);

    // crop depth image
    cv::Rect crop(center_x - size_x / 2, center_y - size_y / 2, size_x, size_y);
    cv::Mat depth_crop = depth_cv_image(crop);
    
    // create range mask
    cv::Mat min_mask = (depth_crop > _min_depth);
    cv::Mat max_mask = (depth_crop < _max_depth);
    min_mask.convertTo(min_mask, CV_8U);
    max_mask.convertTo(max_mask, CV_8U);
    cv::Mat range_mask;
    cv::bitwise_and(min_mask, max_mask, range_mask);

    // apply range mask
    cv::Mat depth_crop_masked;
    depth_crop.copyTo(depth_crop_masked, range_mask);

    // find max value in this masked depth image
    double crop_min, crop_max;
    cv::minMaxLoc(depth_crop, &crop_min, &crop_max);

    // any pixel in the masked depth image that is zero, set to the max value.
    // this assists with normalization
    cv::Mat range_mask_inv;
    cv::bitwise_not(range_mask, range_mask_inv);
    cv::bitwise_or(depth_crop_masked, cv::Scalar(crop_max), depth_crop_masked, range_mask_inv);

    // normalize the masked depth image from min..max to 0..255
    double mask_min, mask_max;
    cv::minMaxLoc(depth_crop_masked, &mask_min, &mask_max);
    cv::Mat normalized;
    depth_crop_masked.convertTo(normalized, CV_8UC1, 255.0 / (mask_max - mask_min), -255.0 * mask_min);

    // apply a binary inverse threshold to the normalized image
    // the result is a foreground mask
    cv::Mat threshold_mask;
    cv::threshold(normalized, threshold_mask, _relative_threshold, 255, cv::THRESH_BINARY_INV);

    // set border to zero then apply erosion
    for (size_t x = 0; x < depth_crop.cols; x++) {
        threshold_mask.at<uchar>(0, x) = 0;
        threshold_mask.at<uchar>(depth_crop.rows - 1, x) = 0;
    }
    for (size_t y = 0; y < depth_crop.rows; y++) {
        threshold_mask.at<uchar>(y, 0) = 0;
        threshold_mask.at<uchar>(y, depth_crop.cols - 1) = 0;
    }
    cv::erode(threshold_mask, threshold_mask, erode_element);

    // if the threshold removed all pixels, reset the mask to all true.
    // this way this function will always return results from the image and not zero
    bool is_mask_empty = false;
    if (countNonZero(threshold_mask) == 0) {
        threshold_mask = cv::Mat::zeros(depth_crop.rows, depth_crop.cols, CV_8UC1);
        cv::rectangle(threshold_mask, cv::Point(crop.x, crop.y), cv::Point(crop.x + crop.width, crop.y + crop.height), cv::Scalar(255), cv::FILLED);
        is_mask_empty = true;
    }

    // find min value in masked depth
    double masked_min, masked_max;
    cv::Point min_loc, max_loc;
    cv::minMaxLoc(depth_crop, &masked_min, &masked_max, &min_loc, &max_loc, threshold_mask);

    // find the mean and standard deviation in the cropped depth image using the foreground mask
    cv::Mat mean, stddev;
    cv::meanStdDev(depth_crop, mean, stddev, threshold_mask);

    // extract scalar values
    double z_dist = mask_min;
    double z_std = stddev.at<double>(0);
    double num_stddevs;

    // set min/max based on mean and standard deviation
    z_min = z_dist;
    if (is_mask_empty) {
        z_max = z_dist + 0.001;
    }
    else {
        z_max = z_dist + z_std * num_stddevs;
    }

    out_mask = cv::Mat::zeros(depth_cv_image.rows, depth_cv_image.cols, CV_8UC1);
    for (size_t x = 0; x < depth_crop.cols; x++) {
        for (size_t y = 0; y < depth_crop.rows; y++) {
            if (threshold_mask.at<uchar>(y, x)) {
                out_mask.at<uchar>(y + crop.y, x + crop.x) = 255;
            }
        }
    }
}

vision_msgs::Detection3D BwYolo::detection_2d_to_3d(vision_msgs::Detection2D detection_2d_msg, double z_min, double z_max)
{
    double z_center = (z_min + z_max) / 2.0;
    double z_size = abs(z_max - z_min);
    int x_center_px = (int)(detection_2d_msg.bbox.center.x);
    int y_center_px = (int)(detection_2d_msg.bbox.center.y);

    int x_size_px = (int)(detection_2d_msg.bbox.size_x);
    int y_size_px = (int)(detection_2d_msg.bbox.size_y);

    cv::Point2d center_point;
    center_point.x = x_center_px;
    center_point.y = y_center_px;
    cv::Point3d ray = _camera_model.projectPixelTo3dRay(center_point);

    double x_center = ray.x * z_center;
    double y_center = ray.y * z_center;

    cv::Point2d edge_point;
    edge_point.x = x_center_px - (int)(x_size_px / 2);
    edge_point.y = y_center_px - (int)(y_size_px / 2);
    ray = _camera_model.projectPixelTo3dRay(edge_point);

    double x_size = std::abs(ray.x * z_center - x_center) * 2.0;
    double y_size = std::abs(ray.y * z_center - y_center) * 2.0;

    geometry_msgs::Pose pose;

    pose.position.x = x_center;
    pose.position.y = y_center;
    pose.position.z = z_center;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    vision_msgs::BoundingBox3D bbox;

    bbox.center = pose;
    bbox.size.x = x_size;
    bbox.size.y = y_size;
    bbox.size.z = z_size;

    vision_msgs::Detection3D detection_3d_msg;
    detection_3d_msg.header = detection_2d_msg.header;
    detection_3d_msg.results = detection_2d_msg.results;

    detection_3d_msg.results[0].pose.pose = bbox.center;
    detection_3d_msg.bbox = bbox;

    return detection_3d_msg;
}

void BwYolo::tf_detection_pose_to_robot(vision_msgs::Detection3D& detection_3d_msg)
{
    if (_target_frame.length() == 0) {
        return;
    }
    geometry_msgs::TransformStamped transform_camera_to_target;

    try {
        transform_camera_to_target = tfBuffer.lookupTransform(
            _target_frame, detection_3d_msg.header.frame_id,
            detection_3d_msg.header.stamp, ros::Duration(1.0)
        );
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = detection_3d_msg.header;
        pose_stamped.pose = detection_3d_msg.bbox.center;
        tf2::doTransform(pose_stamped, pose_stamped, transform_camera_to_target);
        // pose_stamped now contains the object position in the target frame
        detection_3d_msg.header.frame_id = _target_frame;
        detection_3d_msg.bbox.center = pose_stamped.pose;
        detection_3d_msg.results[0].pose.pose = pose_stamped.pose;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(0.5, "%s", ex.what());
        return;
    }
}

std::string BwYolo::get_class_name(int obj_id)
{
    return _class_names[get_class_index(obj_id)];
}

int BwYolo::get_class_index(int obj_id)
{
    return obj_id & 0xffff;
}

int BwYolo::get_class_count(int obj_id)
{
    return obj_id >> 16;
}

vision_msgs::Detection2DArray BwYolo::detections_to_msg(const std::vector<std::vector<Detection>>& detections)
{
    vision_msgs::Detection2DArray detection_2d_arr_msg;
    for (size_t index = 0; index < _obj_count.size(); index++) {
        _obj_count[index] = 0;
    }

    for (const auto& detection : detections[0]) {
        const auto& box = detection.bbox;
        float score = detection.score;
        int class_idx = detection.class_idx;
        if (class_idx >= _obj_count.size()) {
            ROS_ERROR("Class index %d is not a registered class!", class_idx);
            continue;
        }
        int class_count = _obj_count[class_idx];
        _obj_count[class_idx]++;

        int obj_id = (class_count << 16) | class_idx;

        vision_msgs::Detection2D detection_2d_msg;

        detection_2d_msg.bbox.size_x = (double)box.width;
        detection_2d_msg.bbox.size_y = (double)box.height;

        detection_2d_msg.bbox.center.x = (double)box.x + (double)(box.width) / 2.0;
        detection_2d_msg.bbox.center.y = (double)box.y + (double)(box.height) / 2.0;

        detection_2d_msg.bbox.center.theta = 0.0;

        vision_msgs::ObjectHypothesisWithPose hyp;

        hyp.id = obj_id;
        hyp.score = score;

        detection_2d_msg.results.push_back(hyp);
        detection_2d_arr_msg.detections.push_back(detection_2d_msg);
    }
    return detection_2d_arr_msg;
}

bw_yolo::DetectionObjectsStamped BwYolo::convert_to_objects(
    unsigned int width,
    unsigned int height,
    vision_msgs::Detection2DArray detection_2d_arr_msg,
    vision_msgs::Detection3DArray detection_3d_arr_msg)
{
    bw_yolo::DetectionObjectsStamped objects;
    objects.header = detection_3d_arr_msg.header;
    objects.width = width;
    objects.height = height;

    for (size_t index = 0; index < detection_2d_arr_msg.detections.size(); index++) {
        vision_msgs::Detection2D detection_2d_msg = detection_2d_arr_msg.detections[index];
        vision_msgs::Detection3D detection_3d_msg = detection_3d_arr_msg.detections[index];

        bw_yolo::DetectionObject obj;
        obj.label = get_class_name(detection_2d_msg.results[0].id);
        obj.object_index = get_class_count(detection_2d_msg.results[0].id);
        obj.class_index = get_class_index(detection_2d_msg.results[0].id);
        obj.confidence = detection_2d_msg.results[0].score;
        obj.pose = detection_3d_msg.results[0].pose.pose;

        int x_left = (int)(detection_2d_msg.bbox.center.x - detection_2d_msg.bbox.size_x / 2.0);
        int x_right = (int)(detection_2d_msg.bbox.center.x + detection_2d_msg.bbox.size_x / 2.0);
        int y_top = (int)(detection_2d_msg.bbox.center.y - detection_2d_msg.bbox.size_y / 2.0);
        int y_bottom = (int)(detection_2d_msg.bbox.center.y + detection_2d_msg.bbox.size_y / 2.0);

        obj.bounding_box_2d.points[0].x = x_left;
        obj.bounding_box_2d.points[0].y = y_top;
        obj.bounding_box_2d.points[1].x = x_right;
        obj.bounding_box_2d.points[1].y = y_top;
        obj.bounding_box_2d.points[2].x = x_right;
        obj.bounding_box_2d.points[2].y = y_bottom;
        obj.bounding_box_2d.points[3].x = x_left;
        obj.bounding_box_2d.points[3].y = y_bottom;

        double half_x = detection_3d_msg.bbox.size.x / 2.0;
        double half_y = detection_3d_msg.bbox.size.y / 2.0;
        double half_z = detection_3d_msg.bbox.size.z / 2.0;
        for (int index = 0; index < obj.bounding_box_3d.points.size(); index++) {
            obj.bounding_box_3d.points[index].x = _box_point_permutations[index][0] * half_x;
            obj.bounding_box_3d.points[index].y = _box_point_permutations[index][1] * half_y;
            obj.bounding_box_3d.points[index].z = _box_point_permutations[index][2] * half_z;
        }
        obj.bounding_box_3d.dimensions.x = detection_3d_msg.bbox.size.x;
        obj.bounding_box_3d.dimensions.y = detection_3d_msg.bbox.size.y;
        obj.bounding_box_3d.dimensions.z = detection_3d_msg.bbox.size.z;

        objects.objects.push_back(obj);
    }
    return objects;
}

void BwYolo::draw_overlay(cv::Mat img, const std::vector<std::vector<Detection>>& detections, cv::Mat debug_mask, bool label)
{
    if (detections.empty()) {
        return;
    }
    cv::cvtColor(debug_mask, debug_mask, cv::COLOR_GRAY2BGR);
    cv::bitwise_and(img, debug_mask, img);
    for (const auto& detection : detections[0]) {
        const auto& box = detection.bbox;
        float score = detection.score;
        int class_idx = detection.class_idx;

        cv::rectangle(img, box, cv::Scalar(0, 0, 255), 2);

        if (label) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << score;
            std::string s = _class_names[class_idx] + " " + ss.str();

            auto font_face = cv::FONT_HERSHEY_DUPLEX;
            auto font_scale = 1.0;
            int thickness = 1;
            int baseline=0;
            auto s_size = cv::getTextSize(s, font_face, font_scale, thickness, &baseline);
            cv::rectangle(img,
                    cv::Point(box.tl().x, box.tl().y - s_size.height - 5),
                    cv::Point(box.tl().x + s_size.width, box.tl().y),
                    cv::Scalar(0, 0, 255), -1);
            cv::putText(img, s, cv::Point(box.tl().x, box.tl().y - 5),
                        font_face , font_scale, cv::Scalar(255, 255, 255), thickness);
        }
    }
}

void BwYolo::add_detection_to_marker_array(visualization_msgs::MarkerArray& marker_array, vision_msgs::Detection3D detection_3d_msg, std_msgs::ColorRGBA color)
{
    visualization_msgs::Marker cube_marker = make_marker(detection_3d_msg, color);
    visualization_msgs::Marker line_marker = make_marker(detection_3d_msg, color);
    visualization_msgs::Marker text_marker = make_marker(detection_3d_msg, color);

    std::string label = get_class_name(detection_3d_msg.results[0].id);
    int count = get_class_count(detection_3d_msg.results[0].id);

    cube_marker.type = visualization_msgs::Marker::CUBE;
    cube_marker.ns = "box_" + cube_marker.ns;
    cube_marker.color.a = _cube_opacity;

    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.ns = "line_" + line_marker.ns;
    double size_x = detection_3d_msg.bbox.size.x / 2.0;
    double size_y = detection_3d_msg.bbox.size.y / 2.0;
    line_marker.points.push_back(make_point(size_x, size_y, 0.0));
    line_marker.points.push_back(make_point(-size_x, size_y, 0.0));
    line_marker.points.push_back(make_point(-size_x, -size_y, 0.0));
    line_marker.points.push_back(make_point(size_x, -size_y, 0.0));
    line_marker.points.push_back(make_point(size_x, size_y, 0.0));
    line_marker.scale.x = _visuals_line_width;
    line_marker.color.r = 1.0 - text_marker.color.r;
    line_marker.color.g = 1.0 - text_marker.color.g;
    line_marker.color.b = 1.0 - text_marker.color.b;
    line_marker.color.a = 1.0;

    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.ns = "text_" + cube_marker.ns;
    boost::format fmt = boost::format("%s_%s|%0.1f") % label % count % (detection_3d_msg.results[0].score * 100.0);
    text_marker.text = fmt.str();
    text_marker.scale.z = std::min({text_marker.scale.x, text_marker.scale.y});
    text_marker.scale.x = 0.0;
    text_marker.scale.y = 0.0;
    text_marker.color.r = 1.0 - text_marker.color.r;
    text_marker.color.g = 1.0 - text_marker.color.g;
    text_marker.color.b = 1.0 - text_marker.color.b;

    marker_array.markers.push_back(cube_marker);
    marker_array.markers.push_back(line_marker);
    marker_array.markers.push_back(text_marker);
}

visualization_msgs::Marker BwYolo::make_marker(vision_msgs::Detection3D detection_3d_msg, std_msgs::ColorRGBA color)
{
    std::string label = get_class_name(detection_3d_msg.results[0].id);
    int count = get_class_count(detection_3d_msg.results[0].id);

    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = detection_3d_msg.bbox.center;
    marker.header = detection_3d_msg.header;
    marker.lifetime = _marker_persistance;
    marker.ns = label;
    marker.id = count;

    marker.scale = detection_3d_msg.bbox.size;
    marker.color = color;
    marker.frame_locked = false;

    return marker;
}

int BwYolo::run()
{
    ros::spin();

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bw_yolo");
    ros::NodeHandle nh;
    BwYolo node(&nh);
    return node.run();
}
