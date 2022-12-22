#include "bw_tunnel.h"

BwTunnel::BwTunnel(ros::NodeHandle* nodehandle) :
    BwSerialTunnel(nodehandle)
{
    ros::param::param<bool>("~publish_odom_tf", _publish_odom_tf, true);
    ros::param::param<string>("~base_frame", _base_frame, "base_link");
    ros::param::param<string>("~odom_frame", _odom_frame, "odom");
    ros::param::param<string>("~map_frame", _map_frame, "map");

    ros::param::param<double>("~cmd_vel_timeout", _cmd_vel_timeout_param, 0.5);
    ros::param::param<int>("~num_modules", _num_modules, 4);

    string key;
    if (!ros::param::search("joint_names", key)) {
        ROS_ERROR("Failed to find joint_names parameter");
        std::exit(EXIT_FAILURE);
    }
    ROS_DEBUG("Found joint_names: %s", key.c_str());
    nh.getParam(key, _joint_names);

    _cmd_vel_timeout = ros::Duration(_cmd_vel_timeout_param);

    _odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    _odom_msg.header.frame_id = _odom_frame;
    _odom_msg.child_frame_id = _base_frame;
    /* [
        1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1e-3
    ] */
    /* [
         0,  1,  2,  3,  4,  5,
         6,  7,  8,  9, 10, 11,
        12, 13, 14, 15, 16, 17,
        18, 19, 20, 21, 22, 23,
        24, 25, 26, 27, 28, 29,
        30, 31, 32, 33, 34, 35
    ] */
    // odom_msg.pose.covariance.resize(36);
    _odom_msg.pose.covariance[0] = 5e-2;
    _odom_msg.pose.covariance[7] = 5e-2;
    _odom_msg.pose.covariance[14] = 5e-2;
    _odom_msg.pose.covariance[21] = 5e-2;
    _odom_msg.pose.covariance[28] = 5e-2;
    _odom_msg.pose.covariance[35] = 5e-2;

    // odom_msg.twist.covariance.resize(36);
    _odom_msg.twist.covariance[0] = 10e-2;
    _odom_msg.twist.covariance[7] = 10e-2;
    _odom_msg.twist.covariance[14] = 10e-2;
    _odom_msg.twist.covariance[21] = 10e-2;
    _odom_msg.twist.covariance[28] = 10e-2;
    _odom_msg.twist.covariance[35] = 10e-2;

    _raw_joint_pubs = new vector<ros::Publisher>();
    _raw_joint_msgs = new vector<std_msgs::Float64*>();

    _module_pubs = new vector<ros::Publisher>();
    
    for (int index = 0; index < _joint_names.size(); index++) {
        addJointPub(_joint_names.at(index));
    }

    _charge_pub = nh.advertise<bw_interfaces::ChargeState>("charger", 10);
    _button_pub = nh.advertise<std_msgs::Bool>("button_pressed", 10);
    _button_counter_pub = nh.advertise<std_msgs::Int32>("button_counter", 10);
    _button_counter_result_pub = nh.advertise<std_msgs::Int32>("button_counter_result", 10);
    _is_enabled_pub = nh.advertise<std_msgs::Bool>("are_motors_enabled", 10);
    for (int index = 0; index < _num_modules; index++) {
        _module_pubs->push_back(nh.advertise<bw_interfaces::BwDriveModule>("module/" + std::to_string(index + 1), 50));
    }

    _twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 50, &BwTunnel::twistCallback, this);
    _module_sub = nh.subscribe<bw_interfaces::BwDriveModule>("module_command", 50, &BwTunnel::moduleCommandCallback, this);
    _sequence_sub = nh.subscribe<bw_interfaces::BwSequence>("sequence", 50, &BwTunnel::loadSequenceCallback, this);

    _set_enabled_sub = nh.subscribe<std_msgs::Bool>("set_motors_enabled", 10, &BwTunnel::setEnabledCallback, this);

    _prev_twist_timestamp = ros::Time(0);
    _twist_cmd_vx = 0.0;
    _twist_cmd_vy = 0.0;
    _twist_cmd_vt = 0.0;

    _odom_reset_srv = nh.advertiseService("odom_reset_service", &BwTunnel::odomResetCallback, this);
    _play_sequence_srv = nh.advertiseService("play_sequence", &BwTunnel::playSequenceCallback, this);
    _stop_sequence_srv = nh.advertiseService("stop_sequence", &BwTunnel::stopSequenceCallback, this);
    _sequence_state_pub = nh.advertise<bw_interfaces::BwSequenceState>("sequence_state", 10);

    begin();

    ROS_INFO("bw_tunnel init complete");
}

void BwTunnel::addJointPub(string name)
{
    ROS_INFO("Subscribing to joint topic: %s", name.c_str());
    _raw_joint_pubs->push_back(nh.advertise<std_msgs::Float64>("joint/" + name, 50));
    _raw_joint_msgs->push_back(new std_msgs::Float64);
}

void BwTunnel::packetCallback(PacketResult* result)
{
    BwSerialTunnel::packetCallback(result);
    string category = result->getCategory();
    if (category.compare("od") == 0) {
        double x, y, t;
        float vx, vy, vt;
        if (!result->getDouble(x))  { ROS_ERROR("Failed to get odom x"); return; }
        if (!result->getDouble(y))  { ROS_ERROR("Failed to get odom y"); return; }
        if (!result->getDouble(t))  { ROS_ERROR("Failed to get odom t"); return; }
        if (!result->getFloat(vx))  { ROS_ERROR("Failed to get odom vx"); return; }
        if (!result->getFloat(vy))  { ROS_ERROR("Failed to get odom vy"); return; }
        if (!result->getFloat(vt))  { ROS_ERROR("Failed to get odom vt"); return; }
        publishOdom(
            result->getRecvTime(),
            x, y, t, vx, vy, vt
        );
    }
    else if (category.compare("mo") == 0) {
        uint8_t channel;
        float azimuth, wheel_velocity;
        double wheel_position;
        if (!result->getUInt8(channel))  { ROS_ERROR("Failed to get module channel"); return; }
        if (!result->getFloat(azimuth))  { ROS_ERROR("Failed to get module azimuth"); return; }
        if (!result->getDouble(wheel_position))  { ROS_ERROR("Failed to get module wheel position"); return; }
        if (!result->getFloat(wheel_velocity))  { ROS_ERROR("Failed to get module wheel velocity"); return; }
        publishJoint(
            result->getRecvTime(),
            (int)channel, azimuth
        );

        if (channel >= _module_pubs->size()) {
            ROS_WARN_THROTTLE(1.0, "Encountered invalid channel index: %d", (int)channel);
        }

        bw_interfaces::BwDriveModule msg;
        msg.module_index = std::to_string(channel + 1);
        msg.azimuth_position = (double)azimuth;
        msg.wheel_position = wheel_position;
        msg.wheel_velocity = (double)wheel_velocity;
        _module_pubs->at(channel).publish(msg);
    }
    else if (category.compare("power") == 0) {
        float voltage, current;
        bool is_charging;
        if (!result->getFloat(voltage))  { ROS_ERROR("Failed to get voltage"); return; }
        if (!result->getFloat(current))  { ROS_ERROR("Failed to get current"); return; }
        if (!result->getBool(is_charging))  { ROS_ERROR("Failed to get is_charging"); return; }

        bw_interfaces::ChargeState msg;
        msg.battery_voltage = voltage;
        msg.charge_current = current;
        msg.is_charging = is_charging;
        _charge_pub.publish(msg);
    }
    else if (category.compare("bu") == 0) {
        bool button_state;
        if (!result->getBool(button_state))  { ROS_ERROR("Failed to get button_state"); return; }

        std_msgs::Bool msg;
        msg.data = button_state;
        _button_pub.publish(msg);
    }
    else if (category.compare("bp") == 0) {
        int32_t counter;
        if (!result->getInt32(counter))  { ROS_ERROR("Failed to get counter"); return; }

        std_msgs::Int32 msg;
        msg.data = counter;
        _button_counter_pub.publish(msg);
    }
    else if (category.compare("br") == 0) {
        int32_t counter;
        if (!result->getInt32(counter))  { ROS_ERROR("Failed to get counter"); return; }

        std_msgs::Int32 msg;
        msg.data = counter;
        _button_counter_result_pub.publish(msg);
    }
    else if (category.compare("en") == 0) {
        bool enable_state;
        if (!result->getBool(enable_state))  { ROS_ERROR("Failed to get button_state"); return; }

        std_msgs::Bool msg;
        msg.data = enable_state;
        _is_enabled_pub.publish(msg);
    }
    else if (category.compare("|seq") == 0) {
        bool status;
        uint8_t sequence;
        bool is_from_flash;
        uint16_t index;
        if (!result->getBool(status))  { ROS_ERROR("Failed to get status"); return; }
        if (!result->getUInt8(sequence))  { ROS_ERROR("Failed to get sequence"); return; }
        if (!result->getBool(is_from_flash))  { ROS_ERROR("Failed to get is_from_flash"); return; }
        if (!result->getUInt16(index))  { ROS_ERROR("Failed to get index"); return; }

        bw_interfaces::BwSequenceState state;
        state.serial = sequence;
        state.is_from_flash = is_from_flash;
        state.is_running = status;
        state.index = index;
        _sequence_state_pub.publish(state);
    }
}

void BwTunnel::publishOdom(ros::Time recv_time, double x, double y, double t, double vx, double vy, double vt)
{
    if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(t) && std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vt))) {
        ROS_WARN_THROTTLE(1.0, "Odometry values are nan or inf");
        return;
    }

    tf2::Quaternion quat;
    quat.setRPY(0, 0, t);

    geometry_msgs::Quaternion msg_quat = tf2::toMsg(quat);

    _odom_msg.header.stamp = recv_time;
    _odom_msg.pose.pose.position.x = x;
    _odom_msg.pose.pose.position.y = y;
    _odom_msg.pose.pose.orientation = msg_quat;

    _odom_msg.twist.twist.linear.x = vx;
    _odom_msg.twist.twist.linear.y = vy;
    _odom_msg.twist.twist.angular.z = vt;

    if (_publish_odom_tf)
    {
        geometry_msgs::TransformStamped tf_stamped;
        tf_stamped.header.stamp = recv_time;
        tf_stamped.header.frame_id = _odom_frame;
        tf_stamped.child_frame_id = _base_frame;
        tf_stamped.transform.translation.x = x;
        tf_stamped.transform.translation.y = y;
        tf_stamped.transform.translation.z = 0.0;
        tf_stamped.transform.rotation = msg_quat;

        _tf_broadcaster.sendTransform(tf_stamped);
    }
    
    _odom_pub.publish(_odom_msg);
}

void BwTunnel::publishJoint(ros::Time recv_time, int joint_index, double joint_position)
{
    if (joint_index < 0 || joint_index >= _raw_joint_msgs->size()) {
        ROS_WARN("Invalid joint index received: %d. Valid range is 0..%lu. (Joint value was %f. recv time is %f)", joint_index, _raw_joint_msgs->size() - 1, joint_position, recv_time.toSec());
        return;
    }
    if (!std::isfinite(joint_position)) {
        ROS_WARN_THROTTLE(1.0, "Joint position for index %d is nan or inf", joint_index);
        return;
    }
    std_msgs::Float64* msg = _raw_joint_msgs->at(joint_index);
    msg->data = joint_position;

    _raw_joint_pubs->at(joint_index).publish(*msg);
}

void BwTunnel::twistCallback(const geometry_msgs::TwistConstPtr& msg)
{
    _twist_cmd_vx = msg->linear.x;
    _twist_cmd_vy = msg->linear.y;
    _twist_cmd_vt = msg->angular.z;
    _prev_twist_timestamp = ros::Time::now();
}

void BwTunnel::moduleCommandCallback(const bw_interfaces::BwDriveModuleConstPtr& msg)
{
    uint8_t channel = (uint8_t)stoi(msg->module_index);
    float azimuth_position = msg->azimuth_position;
    double wheel_position = msg->wheel_position;
    float wheel_velocity = msg->wheel_velocity;
    writePacket("mo", "cfef", channel, azimuth_position, wheel_position, wheel_velocity);
}

void BwTunnel::loadSequenceCallback(const bw_interfaces::BwSequenceConstPtr& msg)
{
    uint16_t length = 0;
    if (msg->sequence.size() >= 0xffff) {
        ROS_WARN("Sequence length (%lu) exceeds max length. Ignoring the rest.", msg->sequence.size());
        length = 0xffff;
    }
    else {
        length = (uint16_t)msg->sequence.size();
    }
    writePacket("lseq", "cg", msg->serial, length);
    int attempts = 0;
    for (size_t index = 0; index < length; index++) {
        bw_interfaces::BwSequenceElement element = msg->sequence.at(index);
        while (getResult("seq", "cgm", 0.0, 1.0, msg->serial, index, element.parameters) == NULL) {
            attempts++;
            if (attempts > 10) {
                ROS_WARN("Failed to write entire sequence!");
                break;
            }
        }
        attempts = 0;
    }
}

void BwTunnel::setEnabledCallback(const std_msgs::BoolConstPtr& msg)
{
    ROS_INFO("Setting motor enable to %d", msg->data);
    writePacket("en", "b", msg->data);
}

void BwTunnel::publishCmdVel()
{
    ros::Duration dt = ros::Time::now() - _prev_twist_timestamp;
    if (dt > _cmd_vel_timeout) {
        ROS_DEBUG_THROTTLE(5.0, "cmd_vel timed out skipping write.");
        return;
    }

    writePacket("d", "fff", _twist_cmd_vx, _twist_cmd_vy, _twist_cmd_vt);
}

void BwTunnel::writeDeviceTick() {
    publishCmdVel();
}

bool BwTunnel::odomResetCallback(bw_interfaces::OdomReset::Request &req, bw_interfaces::OdomReset::Response &resp)
{
    writePacket("reset", "fff", req.x, req.y, req.t);
    ROS_INFO("Resetting odometry to x: %0.3f, y: %0.3f, theta: %0.3f", req.x, req.y, req.t);
    resp.resp = true;
    return true;
}

bool BwTunnel::playSequenceCallback(bw_interfaces::PlaySequence::Request &req, bw_interfaces::PlaySequence::Response &resp)
{
    ROS_INFO("Playing sequence %d. Looping=%d", req.serial, req.loop);
    // writeHandshakePacket(">seq", "cb", 0.5, 1.0, req.serial, req.loop);
    // resp.success = true;
    PacketResult* result = getResult(">seq", "cbb", 0.0, 1.0, req.serial, req.loop, req.from_flash);
    LOCK_GET_RESULT;
    bool success;
    if (result == NULL || !result->getBool(success)) {
        ROS_WARN("Failed to get start sequence result!");
        resp.success = false;
    }
    else {
        resp.success = success;
    }
    return true;
}

bool BwTunnel::stopSequenceCallback(bw_interfaces::StopSequence::Request &req, bw_interfaces::StopSequence::Response &resp)
{
    ROS_INFO("Stopping sequence.");
    // writeHandshakePacket("xseq", "", 0.0, 1.0);
    // resp.success = true;
    PacketResult* result = getResult("xseq", "", 0.0, 1.0);
    LOCK_GET_RESULT;
    bool success;
    if (result == NULL || !result->getBool(success)) {
        resp.success = false;
    }
    else {
        resp.success = success;
    }
    return true;
}


int BwTunnel::run()
{
    ros::spin();
    this->join();
    return 0;
}
