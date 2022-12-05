#include "bw_serial_tunnel.h"

BwSerialTunnel::BwSerialTunnel(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<string>("~device_path", _device_path, "/dev/ttyTHS0");
    ros::param::param<int>("~device_baud", _device_baud, 115200);

    ros::param::param<double>("~tunnel_rate", _tunnel_rate, 200.0);
    ros::param::param<double>("~write_rate", _write_rate, 15.0);
    ros::param::param<int>("~open_attempts", _open_attempts, 50);

    double last_read_timeout_param;
    ros::param::param<double>("~last_read_timeout", last_read_timeout_param, 5.0);

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
    _last_read_threshold = ros::Duration(last_read_timeout_param);

    _is_waiting_on_result = false;
    _result_get = NULL;

    _ping_pub = nh.advertise<std_msgs::Float64>("ping", 50);

    _packet_count_pub = nh.advertise<std_msgs::Int32>("packet_count", 10);
    _packet_rate_pub = nh.advertise<std_msgs::Float64>("packet_rate", 10);

    _status_prev_time = ros::Time::now();
    _status_prev_count = 0;
    _packet_count = 0;

    _start_time = ros::Time::now();

    
}

void BwSerialTunnel::begin()
{
    _ping_timer = nh.createTimer(ros::Duration(0.5), &BwSerialTunnel::pingCallback, this);

    _poll_thread = new boost::thread(&BwSerialTunnel::pollDeviceTask, this);
    _write_thread = new boost::thread(&BwSerialTunnel::writeDeviceTask, this);
}

bool BwSerialTunnel::reOpenDevice()
{
    for (int attempt = 0; attempt < _open_attempts; attempt++)
    {
        if (!ros::ok()) {
            ROS_INFO("[%s] Exiting reopen", _device_path.c_str());
            break;
        }
        ros::Duration(2.0).sleep();
        if (attempt > 0) {
            ROS_INFO("[%s] Open device attempt #%d", _device_path.c_str(), attempt + 1);
        }
        closeDevice();
        if (openDevice()) {
            break;
        }
        ROS_INFO("[%s] Connection attempt failed", _device_path.c_str());
    }
    if (!_initialized) {
        ROS_ERROR("[%s] Maximum number of attempts reached", _device_path.c_str());
    }
    return _initialized;
}

bool BwSerialTunnel::openDevice()
{
    ROS_INFO("[%s] Initializing device", _device_path.c_str());

    try {
        _device.setPort(_device_path);
        _device.setBaudrate(_device_baud);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        _device.setTimeout(timeout);
        _device.open();
    }
    catch (serial::IOException& e) {
        ROS_WARN("[%s] Device failed to open!", _device_path.c_str());
        return false;
    }
    
    if (!_device.isOpen()) {
        return false;
    }

    _initialized = true;
    ROS_INFO("[%s] Device initialized", _device_path.c_str());
    _last_read_time = ros::Time::now();

    return true;
}
bool BwSerialTunnel::didDeviceTimeout()
{
    if (ros::Time::now() - _last_read_time > _last_read_threshold) {
        ROS_INFO("[%s] Device timed out while waiting for data", _device_path.c_str());
        _last_read_time = ros::Time::now();
        return true;
    }
    else {
        return false;
    }
}


void BwSerialTunnel::packetCallback(PacketResult* result)
{
    _packet_count++;
    string category = result->getCategory();
    if (category.compare("ping") == 0) {
        float ping_time;
        if (!result->getFloat(ping_time))  { ROS_ERROR("Failed to get ping_time"); return; }
        double dt = getLocalTime() - (double)ping_time;
        ROS_DEBUG("[%s] Publishing ping time: %f. (Return time: %f)", _device_path.c_str(), dt, ping_time);
        publishStatusMessages(dt);
    }
}

double BwSerialTunnel::getLocalTime() {
    return (ros::Time::now() - _start_time).toSec();
}

void BwSerialTunnel::publishStatusMessages(float ping)
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

void BwSerialTunnel::pingCallback(const ros::TimerEvent& event) {
    double ping_time = getLocalTime();
    ROS_DEBUG("[%s] Writing ping time: %f", _device_path.c_str(), ping_time);
    writePacket("ping", "f", ping_time);
}


void BwSerialTunnel::writePacket(string category, const char *formats, ...)
{
    if (!_initialized) {
        ROS_DEBUG("[%s] Device is not initialized. Skipping write. Category: %s", _device_path.c_str(), category.c_str());
        return;
    }
    va_list args;
    va_start(args, formats);
    unsigned int length = _protocol->makePacket(PACKET_TYPE_NORMAL, _write_buffer, category, formats, args);
    // ROS_DEBUG("Writing packet: %s", packetToString(_write_buffer, 0, length).c_str());
    if (length > 0) {
        _write_lock.lock();
        try {
            _device.write((uint8_t*)_write_buffer, length);
        }
        catch (serial::IOException& e) {
            ROS_WARN("[%s] Failed to write to device!", _device_path.c_str());
        }
        _write_lock.unlock();
    }
    else {
        ROS_DEBUG("[%s] Skipping write for packet: %s. Length is %d", _device_path.c_str(), packetToString(_write_buffer, 0, length).c_str(), length);
    }
    va_end(args);
}

void BwSerialTunnel::writeHandshakePacket(string category, const char *formats, double write_interval, double timeout, ...)
{
    if (!_initialized) {
        ROS_DEBUG("[%s] Device is not initialized. Skipping write. Category: %s", _device_path.c_str(), category.c_str());
        return;
    }
    va_list args;
    va_start(args, timeout);
    int length = _protocol->makePacket(PACKET_TYPE_HANDSHAKE, _write_buffer, category, formats, args);
    // ROS_DEBUG("Writing handshake packet: %s", packetToString(_write_buffer, 0, length).c_str());
    if (length > 0) {
        Handshake* handshake = new Handshake(category, _write_buffer, length, _protocol->getWritePacketNum() - 1, write_interval, timeout);
        _handshake_lock.lock();
        _pending_handshakes.push_back(handshake);
        _handshake_lock.unlock();
        _write_lock.lock();
        try {
            _device.write((uint8_t*)_write_buffer, length);
        }
        catch (serial::IOException& e) {
            ROS_WARN("[%s] Failed to write to device!", _device_path.c_str());
        }
        _write_lock.unlock();
    }
    else {
        ROS_DEBUG("[%s] Skipping write for packet: %s. Length is %d", _device_path.c_str(), packetToString(_write_buffer, 0, length).c_str(), length);
    }
    va_end(args);
}

PacketResult* BwSerialTunnel::getResult(string category, const char *formats, double write_interval, double timeout, ...)
{
    if (!_initialized) {
        ROS_DEBUG("[%s] Device is not initialized. Skipping write. Category: %s", _device_path.c_str(), category.c_str());
        return NULL;
    }
    va_list args;
    va_start(args, timeout);
    int length = _protocol->makePacket(PACKET_TYPE_HANDSHAKE, _write_buffer, category, formats, args);
    // ROS_INFO("Writing get packet: %s", packetToString(_write_buffer, 0, length).c_str());
    PacketResult* result = NULL;
    if (length > 0) {
        Handshake* handshake = new Handshake(category, _write_buffer, length, _protocol->getWritePacketNum() - 1, write_interval, timeout);
        _handshake_lock.lock();
        _pending_handshakes.push_back(handshake);
        _handshake_lock.unlock();
        _write_lock.lock();
        try {
            _device.write((uint8_t*)_write_buffer, length);
        }
        catch (serial::IOException& e) {
            ROS_WARN("[%s] Failed to write to device!", _device_path.c_str());
            return new PacketResult(TunnelProtocol::NULL_ERROR, ros::Time::now());
        }
        _write_lock.unlock();

        std::unique_lock<std::mutex> lk(_result_get_lock);
        _is_waiting_on_result = true;
        if (_result_get == NULL) {
            _result_get = new PacketResult(TunnelProtocol::NULL_ERROR, ros::Time::now());
        }
        _result_get->setCategory(category);
        std::chrono::duration<double> cond_timeout(timeout);
        _result_get_condition.wait_for(lk, cond_timeout, [this]{ return !this->_is_waiting_on_result; });
        result = _result_get;
    }
    else {
        ROS_DEBUG("[%s] Skipping write for packet: %s. Length is %d", _device_path.c_str(), packetToString(_write_buffer, 0, length).c_str(), length);
    }
    va_end(args);
    return result;
}

void BwSerialTunnel::checkHandshakes()
{
    for (size_t index = 0; index < _pending_handshakes.size(); index++) {
        Handshake* handshake = _pending_handshakes.at(index);
        if (handshake->didFail()) {
            ROS_WARN("[%s] Handshake packet failed! The category was %s.", _device_path.c_str(), handshake->getCategory().c_str());
            _handshake_lock.lock();
            delete _pending_handshakes.at(index);
            _pending_handshakes.erase(_pending_handshakes.begin() + index);
            _handshake_lock.unlock();
            index--;
            break;
        }
        if (handshake->shouldWriteAgain()) {
            ROS_DEBUG("[%s] Writing a handshake packet again", _device_path.c_str());
            _write_lock.lock();
            unsigned int length = handshake->getPacket(_write_buffer);
            try {
                _device.write((uint8_t*)_write_buffer, length);
            }
            catch (serial::IOException& e) {
                ROS_WARN("[%s] Failed to write to device!", _device_path.c_str());
                return;
            }
            _write_lock.unlock();
        }
    }
}
void BwSerialTunnel::compareHandshakes(PacketResult* result)
{
    ROS_DEBUG("[%s] Confirming packet for %s received.", _device_path.c_str(), result->getCategory().c_str());
    Handshake* handshake = new Handshake(result);
    if (_protocol->isCodeError(result->getErrorCode())) {
        ROS_ERROR("[%s] Handshake %s confirm has an error code: %d.", _device_path.c_str(), handshake->getCategory().c_str(), result->getErrorCode());
        return;
    }
    for (size_t index = 0; index < _pending_handshakes.size(); index++) {
        Handshake* pending_handshake = _pending_handshakes.at(index);
        if (pending_handshake->isEqual(handshake)) {
            ROS_INFO("[%s] Handshake for %s #%d received.", _device_path.c_str(), handshake->getCategory().c_str(), handshake->getPacketNum());
            _handshake_lock.lock();
            delete _pending_handshakes.at(index);
            _pending_handshakes.erase(_pending_handshakes.begin() + index);
            _handshake_lock.unlock();
            index--;
            return;
        }
    }
    ROS_WARN("[%s] Received confirm handshake %s, but no handshakes are expecting it!", _device_path.c_str(), handshake->getCategory().c_str());
}

bool BwSerialTunnel::pollDevice()
{
    if (!_initialized) {
        ROS_WARN("[%s] Device is not initialized.", _device_path.c_str());
        reOpenDevice();
        return true;
    }

    int num_chars_read;
    try {
        num_chars_read = _device.available();
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
    }
    catch (serial::IOException& e) {
        ROS_WARN("[%s] Failed to read from device!", _device_path.c_str());
        return true;
    }
    
    _last_read_time = ros::Time::now();
    int read_stop_index = _unparsed_index + num_chars_read;
    int last_parsed_index = _protocol->parseBuffer(_read_buffer, 0, read_stop_index);

    PacketResult* result = NULL;
    do {
        result = _protocol->popResult();
        if (result->getErrorCode() == TunnelProtocol::NULL_ERROR) {
            continue;
        }
        if (_protocol->isCodeError(result->getErrorCode())) {
            ROS_ERROR("[%s] Encountered error code %d.", _device_path.c_str(), result->getErrorCode());
            continue;
        }
        if (result->getPacketType() == PACKET_TYPE_CONFIRMING) {
            compareHandshakes(result);
        }
        else {
            string category = result->getCategory();
            if (_is_waiting_on_result && _result_get != NULL && _result_get->getCategory().compare(category) == 0)
            {
                {
                    std::lock_guard<std::mutex> lk(_result_get_lock);
                    _is_waiting_on_result = false;
                    _result_get = result;
                }
                _result_get_condition.notify_all();
            }
            packetCallback(result);
        }
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

    checkHandshakes();

    return true;
}

void BwSerialTunnel::pollDeviceTask()
{ 
    ros::Rate clock_rate(_tunnel_rate);  // Hz

    while (ros::ok())
    {
        if (!pollDevice()) {
            ROS_INFO("[%s] Exiting device poll thread", _device_path.c_str());
            break;
        }
        clock_rate.sleep();
    }
    closeDevice();
}

void BwSerialTunnel::writeDeviceTask()
{
    ros::Rate clock_rate(_write_rate);  // Hz
    while (ros::ok())
    {
        writeDeviceTick();
        clock_rate.sleep();
    }
}

void BwSerialTunnel::writeDeviceTick() {

}

void BwSerialTunnel::closeDevice()
{
    try {
        _device.close();
    }
    catch (serial::IOException& e) {
        ROS_WARN("[%s] Failed to close device!", _device_path.c_str());
    }
    _initialized = false;
}


void BwSerialTunnel::join() {
    _poll_thread->join();
}
