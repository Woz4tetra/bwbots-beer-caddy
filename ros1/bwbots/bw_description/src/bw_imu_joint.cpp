#include "bw_imu_joint.h"

BwImuJoint::BwImuJoint(ros::NodeHandle* nodehandle) :
    nh(*nodehandle), _tf_listener(_tf_buffer)
{
    ros::param::param<double>("~publish_rate", _publish_rate, 30.0);
    ros::param::param<string>("~base_parent_frame", _base_parent_frame, "base_link");
    ros::param::param<string>("~base_child_frame", _base_child_frame, "base_tilt_link");
    ros::param::param<string>("~base_imu_frame", _base_imu_frame, "imu");
    ros::param::param<bool>("~combine_with_odom", _combine_with_odom, false);
    ros::param::param<double>("~time_delta_limit", _time_delta_limit, 0.25);

    _static_imu_tf_set = false;
    
    _base_imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 10, &BwImuJoint::base_imu_callback, this);
    if (_combine_with_odom) {
        _odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, &BwImuJoint::odom_callback, this);
        _combined_odom_pub = nh.advertise<nav_msgs::Odometry>("odom/filtered", 50);
    }
    _base_quat_msg.w = 1.0;
    _imu_msg.orientation.w = 1.0;
    x = 0.0;
    y = 0.0;
    theta = 0.0;
}

void BwImuJoint::base_imu_callback(const sensor_msgs::ImuConstPtr& imu)
{
    _imu_msg = *imu;
    tf2::Quaternion quat;
    tf2::convert(imu->orientation, quat);
    base_callback(quat);
}

void BwImuJoint::reset_odom() {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
}

void BwImuJoint::odom_callback(const nav_msgs::OdometryConstPtr& odom) {
    double delta_time = (odom->header.stamp - _prev_odom_time).toSec();
    _prev_odom_time = odom->header.stamp;
    if (delta_time > _time_delta_limit || _prev_odom_time > odom->header.stamp) {
        ROS_WARN("Odometry timestamp jumped! Resetting.");
        reset_odom();
        return;
    }

    double vx = odom->twist.twist.linear.x;
    double vy = odom->twist.twist.linear.y;
    double vt = _imu_msg.angular_velocity.z;

    double dx = vx * delta_time;
    double dy = vy * delta_time;
    double dtheta = vt * delta_time;
    double sin_theta = sin(dtheta);
    double cos_theta = cos(dtheta);

    double s, c;
    if (abs(dtheta) < 1E-9) {  // if angle is too small, use taylor series linear approximation
        s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
        c = 0.5 * dtheta;
    }
    else {
        s = sin_theta / dtheta;
        c = (1.0 - cos_theta) / dtheta;
    }

    double tx = dx * s - dy * c;
    double ty = dx * c + dy * s;

    tf2::Quaternion quat_tf;
    tf2::fromMsg(_imu_msg.orientation, quat_tf);
    tf2::Matrix3x3 m1(quat_tf);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    theta = yaw;

    double rotated_tx = tx * cos(theta) - ty * sin(theta);
    double rotated_ty = tx * sin(theta) + ty * cos(theta);

    x += rotated_tx;
    y += rotated_ty;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(theta)) {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        ROS_WARN("Resetting odometry position! NaN or Inf encountered");
    }

    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header = odom->header;
    tf_stamped.child_frame_id = odom->child_frame_id;
    tf_stamped.transform.translation.x = x;
    tf_stamped.transform.translation.y = y;
    tf_stamped.transform.translation.z = 0.0;
    tf_stamped.transform.rotation = _imu_msg.orientation;
    _tf_broadcaster.sendTransform(tf_stamped);

    nav_msgs::Odometry combined;
    combined.header = odom->header;
    combined.child_frame_id = odom->child_frame_id;
    combined.pose = odom->pose;
    combined.twist = odom->twist;
    combined.pose.pose.orientation = _imu_msg.orientation;
    _combined_odom_pub.publish(combined);
}

void BwImuJoint::base_callback(tf2::Quaternion quat)
{
    if (!_static_imu_tf_set) {
        try {
            _static_imu_to_base_tf = _tf_buffer.lookupTransform(_base_imu_frame, _base_child_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "%s", ex.what());
            return;
        }
        tf2::convert(_static_imu_to_base_tf.transform.rotation, _static_imu_to_base_quat);
        _static_imu_to_base_mat.setRotation(_static_imu_to_base_quat);
        _static_imu_tf_set = true;
    }

    tf2::Matrix3x3 m1(quat);

    m1 *= _static_imu_to_base_mat;

    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    _base_to_base_tilt_quat.setRPY(roll, pitch, yaw);

    // Yaw is set by odom so it's always zero here
    _base_to_base_tilt_quat.setRPY(roll, pitch, 0.0);
    tf2::convert(_base_to_base_tilt_quat, _base_quat_msg);
}

void BwImuJoint::publish_base_tf()
{
    if (!(std::isfinite(_base_quat_msg.x) && std::isfinite(_base_quat_msg.y) && std::isfinite(_base_quat_msg.z) && std::isfinite(_base_quat_msg.w))) {
        ROS_WARN_THROTTLE(1.0, "Base tilt quaternion contains NaNs or Infs!");
        return;
    }
    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header.stamp = ros::Time::now();
    tf_stamped.header.frame_id = _base_parent_frame;
    tf_stamped.child_frame_id = _base_child_frame;
    tf_stamped.transform.translation.x = 0.0;
    tf_stamped.transform.translation.y = 0.0;
    tf_stamped.transform.translation.z = 0.0;
    tf_stamped.transform.rotation = _base_quat_msg;

    _tf_broadcaster.sendTransform(tf_stamped);
}

int BwImuJoint::run()
{
    ros::Rate clock_rate(_publish_rate);  // Hz
    while (ros::ok())
    {
        clock_rate.sleep();
        publish_base_tf();
        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bw_imu_joint");
    ros::NodeHandle nh;
    BwImuJoint node(&nh);
    return node.run();
}
