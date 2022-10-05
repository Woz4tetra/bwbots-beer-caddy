#pragma once

#include <stdio.h>
#include <string.h>
#include <mutex>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "ros/console.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

#include "serial/serial.h"

#include "bw_interfaces/LoadCell.h"

#include "tunnel_protocol.h"


class BwLoadCell {
private:
    ros::NodeHandle nh;  // ROS node handle

    // Parameters
    string _device_path;
    int _device_baud;
    int _open_attempts;

    double _tunnel_rate;  // Hz
    double _write_rate;  // Hz

    // Members
    serial::Serial _device;

    const int READ_BUFFER_LEN = 4096;
    char* _read_buffer;
    bool _initialized;

    int _unparsed_index;
    char* _write_buffer;

    std::mutex _write_lock;
    boost::thread* _poll_thread;

    TunnelProtocol* _protocol;

    ros::Time _last_read_time;
    ros::Duration _last_read_threshold;

    ros::Timer _ping_timer;
    ros::Time _prev_ping_time;
    ros::Duration _ping_interval;

    int _packet_count;
    int _status_prev_count;
    ros::Time _status_prev_time;
    ros::Time _start_time;

    // Publishers
    ros::Publisher _packet_count_pub;
    ros::Publisher _packet_rate_pub;
    ros::Publisher _ping_pub;
    ros::Publisher _load_cell_pub;

    bool reOpenDevice();
    bool openDevice();
    void closeDevice();
    bool didDeviceTimeout();

    void writePacket(string category, const char *formats, ...);

    void packetCallback(PacketResult* result);
    void pingCallback(const ros::TimerEvent& event);
    double getLocalTime();
    void publishStatusMessages(float ping);

    void pollDeviceTask();
    bool pollDevice();
public:
    BwLoadCell(ros::NodeHandle* nodehandle);
    int run();
};