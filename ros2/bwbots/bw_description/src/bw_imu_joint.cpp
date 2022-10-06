#include "bw_imu_joint.h"

BwImuJoint::BwImuJoint(ros::NodeHandle* nodehandle) :
    nh(*nodehandle), _tf_listener(_tf_buffer)
{
    ros::param::param<double>("~publish_rate", _publish_rate, 30.0);
    ros::param::param<string>("~base_parent_frame", _base_parent_frame, "base_link");
    ros::param::param<string>("~base_child_frame", _base_child_frame, "base_tilt_link");
    ros::param::param<string>("~base_imu_frame", _base_imu_frame, "imu");

    _static_imu_tf_set = false;
    
    _base_imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 10, &BwImuJoint::base_imu_callback, this);
    base_quat_msg.w = 1.0;
}

void BwImuJoint::base_imu_callback(const sensor_msgs::ImuConstPtr& imu)
{
    tf2::Quaternion quat;
    tf2::convert(imu->orientation, quat);
    base_callback(quat);
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
    tf2::convert(_base_to_base_tilt_quat, base_quat_msg);
}

void BwImuJoint::publish_base_tf()
{
    if (!(std::isfinite(base_quat_msg.x) && std::isfinite(base_quat_msg.y) && std::isfinite(base_quat_msg.z) && std::isfinite(base_quat_msg.w))) {
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
    tf_stamped.transform.rotation = base_quat_msg;

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
