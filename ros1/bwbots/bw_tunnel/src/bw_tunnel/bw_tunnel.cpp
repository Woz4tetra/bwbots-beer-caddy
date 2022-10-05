#include "bw_tunnel/bw_tunnel.h"

BwTunnel::BwTunnel(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<string>("~device_path", _device_path, "/dev/ttyTHS0");
    ros::param::param<int>("~device_baud", _device_baud, 115200);

    ros::param::param<double>("~tunnel_rate", _tunnel_rate, 200.0);
    ros::param::param<double>("~write_rate", _write_rate, 15.0);

    ros::param::param<bool>("~publish_odom_tf", _publish_odom_tf, true);
    ros::param::param<string>("~base_frame", _base_frame, "base_link");
    ros::param::param<string>("~odom_frame", _odom_frame, "odom");
    ros::param::param<string>("~map_frame", _map_frame, "map");

    ros::param::param<double>("~cmd_vel_timeout", _cmd_vel_timeout_param, 0.5);

    ros::param::param<int>("~open_attempts", _open_attempts, 50);

    string key;
    if (!ros::param::search("joint_names", key)) {
        ROS_ERROR("Failed to find joint_names parameter");
        std::exit(EXIT_FAILURE);
    }
    ROS_DEBUG("Found joint_names: %s", key.c_str());
    nh.getParam(key, _joint_names);

    _cmd_vel_timeout = ros::Duration(_cmd_vel_timeout_param);

    _write_buffer = new char[TunnelProtocol::MAX_PACKET_LEN];
    _read_buffer = new char[READ_BUFFER_LEN];
    _initialized = false;

    _protocol = new TunnelProtocol();

    if (!reOpenDevice()) {
        throw std::runtime_error("Failed to open device");
    }

    _unparsed_index = 0;

    _prev_ping_time = ros::Time(0);
    _ping_interval = ros::Duration(1.0);

    _last_read_time = ros::Time(0);
    _last_read_threshold = ros::Duration(5.0);

    _ping_pub = nh.advertise<std_msgs::Float64>("ping", 50);

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
    
    for (int index = 0; index < _joint_names.size(); index++) {
        addJointPub(_joint_names.at(index));
    }

    _packet_count_pub = nh.advertise<std_msgs::Int32>("packet_count", 10);
    _packet_rate_pub = nh.advertise<std_msgs::Float64>("packet_rate", 10);

    _charge_pub = nh.advertise<bw_interfaces::ChargeState>("charger", 10);
    _button_pub = nh.advertise<std_msgs::Bool>("button_pressed", 10);
    _is_enabled_pub = nh.advertise<std_msgs::Bool>("are_motors_enabled", 10);

    _twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 50, &BwTunnel::twistCallback, this);

    _set_enabled_sub = nh.subscribe<std_msgs::Bool>("set_motors_enabled", 10, &BwTunnel::setEnabledCallback, this);

    _prev_twist_timestamp = ros::Time(0);
    _twist_cmd_vx = 0.0;
    _twist_cmd_vy = 0.0;
    _twist_cmd_vt = 0.0;

    _status_prev_time = ros::Time::now();
    _status_prev_count = 0;
    _packet_count = 0;

    _start_time = ros::Time::now();

    _odom_reset_srv = nh.advertiseService("odom_reset_service", &BwTunnel::odom_reset_callback, this);

    _ping_timer = nh.createTimer(ros::Duration(0.5), &BwTunnel::pingCallback, this);

    _poll_thread = new boost::thread(&BwTunnel::pollDeviceTask, this);
    _write_thread = new boost::thread(&BwTunnel::writeDeviceTask, this);

    ROS_INFO("bw_tunnel init complete");
}

void BwTunnel::addJointPub(string name)
{
    ROS_INFO("Subscribing to joint topic: %s", name.c_str());
    _raw_joint_pubs->push_back(nh.advertise<std_msgs::Float64>("joint/" + name, 50));
    _raw_joint_msgs->push_back(new std_msgs::Float64);
}

bool BwTunnel::reOpenDevice()
{
    for (int attempt = 0; attempt < _open_attempts; attempt++)
    {
        if (!ros::ok()) {
            ROS_INFO("Exiting reopen");
            break;
        }
        ros::Duration(2.0).sleep();
        if (attempt > 0) {
            ROS_INFO("Open device attempt #%d", attempt + 1);
        }
        closeDevice();
        if (openDevice()) {
            break;
        }
        ROS_INFO("Connection attempt failed");
    }
    if (!_initialized) {
        ROS_ERROR("Maximum number of attempts reached");
    }
    return _initialized;
}

bool BwTunnel::openDevice()
{
    ROS_INFO("Initializing device");

    _device.setPort(_device_path);
    _device.setBaudrate(_device_baud);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    _device.setTimeout(timeout);
    _device.open();

    if (!_device.isOpen()) {
        return false;
    }

    _initialized = true;
    ROS_INFO("Device initialized");
    _last_read_time = ros::Time::now();

    return true;
}
bool BwTunnel::didDeviceTimeout()
{
    if (ros::Time::now() - _last_read_time > _last_read_threshold) {
        ROS_INFO("Device timed out while waiting for data");
        _last_read_time = ros::Time::now();
        return true;
    }
    else {
        return false;
    }
}


void BwTunnel::packetCallback(PacketResult* result)
{
    _packet_count++;
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
    else if (category.compare("ping") == 0) {
        float ping_time;
        if (!result->getFloat(ping_time))  { ROS_ERROR("Failed to get ping_time"); return; }
        double dt = getLocalTime() - (double)ping_time;
        ROS_DEBUG("Publishing ping time: %f. (Return time: %f)", dt, ping_time);
        publishStatusMessages(dt);
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
    else if (category.compare("en") == 0) {
        bool enable_state;
        if (!result->getBool(enable_state))  { ROS_ERROR("Failed to get button_state"); return; }

        std_msgs::Bool msg;
        msg.data = enable_state;
        _is_enabled_pub.publish(msg);
    }
}

double BwTunnel::getLocalTime() {
    return (ros::Time::now() - _start_time).toSec();
}

void BwTunnel::publishStatusMessages(float ping)
{
    std_msgs::Float64 ping_msg;
    ping_msg.data = ping;
    _ping_pub.publish(ping_msg);

    int num_messages = _packet_count - _status_prev_count;
    ros::Duration status_interval = ros::Time::now() - _status_prev_time;
    double rate = (double)num_messages / status_interval.toSec();

    std_msgs::Int32 count_msg;
    count_msg.data = _packet_count;
    _packet_count_pub.publish(count_msg);

    std_msgs::Float64 rate_msg;
    rate_msg.data = rate;
    _packet_rate_pub.publish(rate_msg);

    _status_prev_count = _packet_count;
    _status_prev_time = ros::Time::now();
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


void BwTunnel::pingCallback(const ros::TimerEvent& event) {
    double ping_time = getLocalTime();
    ROS_DEBUG("Writing ping time: %f", ping_time);
    writePacket("ping", "f", ping_time);
}


void BwTunnel::writePacket(string category, const char *formats, ...)
{
    if (!_initialized) {
        ROS_DEBUG("Device is not initialized. Skipping write. Category: %s", category.c_str());
        return;
    }
    va_list args;
    va_start(args, formats);
    int length = _protocol->makePacket(PACKET_TYPE_NORMAL, _write_buffer, category, formats, args);
    // ROS_DEBUG("Writing packet: %s", packetToString(_write_buffer, 0, length).c_str());
    if (length > 0) {
        _write_lock.lock();
        _device.write((uint8_t*)_write_buffer, length);
        _write_lock.unlock();
    }
    else {
        ROS_DEBUG("Skipping write for packet: %s. Length is %d", packetToString(_write_buffer, 0, length).c_str(), length);
    }
    va_end(args);
}


bool BwTunnel::pollDevice()
{
    if (!_initialized) {
        ROS_WARN("Device is not initialized.");
        reOpenDevice();
        return true;
    }

    int num_chars_read = _device.available();
    if (num_chars_read <= 0) {
        if (didDeviceTimeout()) {
            reOpenDevice();
        }
        return true;
    }
    if (_unparsed_index + num_chars_read >= READ_BUFFER_LEN) {
        num_chars_read = READ_BUFFER_LEN - _unparsed_index - 1;
    }
    _device.read((uint8_t*)(_read_buffer + _unparsed_index), num_chars_read);

    _last_read_time = ros::Time::now();
    int read_stop_index = _unparsed_index + num_chars_read;
    int last_parsed_index = _protocol->parseBuffer(_read_buffer, 0, read_stop_index);

    PacketResult* result;
    do {
        result = _protocol->popResult();
        if (result->getErrorCode() == TunnelProtocol::NULL_ERROR) {
            continue;
        }
        if (_protocol->isCodeError(result->getErrorCode())) {
            ROS_ERROR("Encountered error code %d.", result->getErrorCode());
            continue;
        }
        string category = result->getCategory();
        packetCallback(result);
        delete result;
    }
    while (result->getErrorCode() != TunnelProtocol::NULL_ERROR);

    _unparsed_index = read_stop_index - last_parsed_index;
    if (_unparsed_index >= READ_BUFFER_LEN) {
        _unparsed_index = 0;
    }

    if (last_parsed_index > 0) {
        for (int index = last_parsed_index, shifted_index = 0; index < READ_BUFFER_LEN; index++, shifted_index++) {
            _read_buffer[shifted_index] = _read_buffer[index];
        }
    }

    return true;
}

void BwTunnel::pollDeviceTask()
{ 
    ros::Rate clock_rate(_tunnel_rate);  // Hz

    while (ros::ok())
    {
        if (!pollDevice()) {
            ROS_INFO("Exiting device poll thread");
            break;
        }
        clock_rate.sleep();
    }
    closeDevice();
}

void BwTunnel::writeDeviceTask()
{
    ros::Rate clock_rate(_write_rate);  // Hz
    while (ros::ok())
    {
        publishCmdVel();
        clock_rate.sleep();
    }
}

void BwTunnel::closeDevice()
{
    _device.close();
    _initialized = false;
}

bool BwTunnel::odom_reset_callback(bw_interfaces::OdomReset::Request &req, bw_interfaces::OdomReset::Response &resp)
{
    writePacket("reset", "fff", req.x, req.y, req.t);
    ROS_INFO("Resetting odometry to x: %0.3f, y: %0.3f, theta: %0.3f", req.x, req.y, req.t);
    resp.resp = true;
    return true;
}

int BwTunnel::run()
{
    ros::spin();
    _poll_thread->join();
    return 0;
}
