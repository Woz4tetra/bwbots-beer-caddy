#include "bw_load_cell.h"

BwLoadCell::BwLoadCell(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<string>("~device_path", _device_path, "/dev/ttyUSB0");
    ros::param::param<int>("~device_baud", _device_baud, 9600);

    ros::param::param<double>("~tunnel_rate", _tunnel_rate, 200.0);
    ros::param::param<double>("~write_rate", _write_rate, 15.0);

    ros::param::param<int>("~open_attempts", _open_attempts, 50);

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

    _status_prev_time = ros::Time::now();
    _status_prev_count = 0;
    _packet_count = 0;

    _start_time = ros::Time::now();

    _ping_pub = nh.advertise<std_msgs::Float64>("load_cell/ping", 50);
    _packet_count_pub = nh.advertise<std_msgs::Int32>("load_cell/packet_count", 10);
    _packet_rate_pub = nh.advertise<std_msgs::Float64>("load_cell/packet_rate", 10);
    _load_cell_pub = nh.advertise<bw_interfaces::LoadCell>("load_cell", 50);

    _ping_timer = nh.createTimer(ros::Duration(0.5), &BwLoadCell::pingCallback, this);
    _poll_thread = new boost::thread(&BwLoadCell::pollDeviceTask, this);

    ROS_INFO("bw_load_cell init complete");
}

bool BwLoadCell::reOpenDevice()
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

bool BwLoadCell::openDevice()
{
    ROS_INFO("Initializing device");

    _device.setPort(_device_path);
    _device.setBaudrate(_device_baud);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    _device.setTimeout(timeout);
    _device.open();

    if (!_device.isOpen()) {
        ROS_INFO("Device failed to initialized");
        return false;
    }

    _initialized = true;
    ROS_INFO("Device initialized");
    _last_read_time = ros::Time::now();
    ros::Duration(2.0).sleep();  // wait for device to boot

    return true;
}

bool BwLoadCell::didDeviceTimeout()
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

void BwLoadCell::closeDevice()
{
    _device.close();
    _initialized = false;
}


void BwLoadCell::writePacket(string category, const char *formats, ...)
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

bool BwLoadCell::pollDevice()
{
    if (!_initialized) {
        ROS_WARN("Device is not initialized.");
        reOpenDevice();
        return true;
    }

    // int num_chars_read = _device.available();
    // if (num_chars_read <= 0) {
    //     if (didDeviceTimeout()) {
    //         reOpenDevice();
    //     }
    //     return true;
    // }
    // if (_unparsed_index + num_chars_read >= READ_BUFFER_LEN) {
    //     num_chars_read = READ_BUFFER_LEN - _unparsed_index - 1;
    // }
    int num_chars_read = (int)_device.read((uint8_t*)(_read_buffer + _unparsed_index), 1);

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

void BwLoadCell::pingCallback(const ros::TimerEvent& event) {
    double ping_time = getLocalTime();
    ROS_DEBUG("Writing ping time: %f", ping_time);
    writePacket("ping", "f", ping_time);
}

double BwLoadCell::getLocalTime() {
    return (ros::Time::now() - _start_time).toSec();
}

void BwLoadCell::publishStatusMessages(float ping)
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
void BwLoadCell::packetCallback(PacketResult* result)
{
    _packet_count++;
    string category = result->getCategory();
    if (category.compare("g") == 0) {
        float load_cell_value;
        if (!result->getFloat(load_cell_value))  { ROS_ERROR("Failed to get load_cell_value"); return; }

        bw_interfaces::LoadCell lc_msg;
        lc_msg.force = (double)load_cell_value;
        _load_cell_pub.publish(lc_msg);
    }
    else if (category.compare("ping") == 0) {
        float ping_time;
        if (!result->getFloat(ping_time))  { ROS_ERROR("Failed to get ping_time"); return; }
        double dt = getLocalTime() - (double)ping_time;
        ROS_DEBUG("Publishing ping time: %f. (Return time: %f)", dt, ping_time);
        publishStatusMessages(dt);
    }
}

void BwLoadCell::pollDeviceTask()
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

int BwLoadCell::run()
{
    ros::spin();
    _poll_thread->join();
    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bw_load_cell");
    ros::NodeHandle nh;

    BwLoadCell broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
