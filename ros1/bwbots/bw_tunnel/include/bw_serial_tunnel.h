#pragma once


#include <stdio.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <string.h>
#include <mutex>
#include <condition_variable>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "ros/console.h"
#include "serial/serial.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

#include "tunnel_protocol.h"

using namespace std;

#define LOCK_GET_RESULT std::unique_lock<std::mutex> lk(_result_get_lock)

class BwSerialTunnel {
private:
    // Parameters
    string _device_path;
    int _device_baud;
    int _open_attempts;

    double _tunnel_rate;  // Hz
    double _write_rate;  // Hz

    // Members
    const int READ_BUFFER_LEN = 4096;
    char* _read_buffer;
    bool _initialized;

    int _unparsed_index;
    char* _write_buffer;

    std::mutex _write_lock;
    std::mutex _handshake_lock;
    boost::thread* _poll_thread;
    boost::thread* _write_thread;
    TunnelProtocol* _protocol;
    serial::Serial _device;

    vector<Handshake*> _pending_handshakes;

    ros::Time _last_read_time;
    ros::Duration _last_read_threshold;

    ros::Timer _ping_timer;
    ros::Time _prev_ping_time;
    ros::Duration _ping_interval;

    int _packet_count;
    int _status_prev_count;
    ros::Time _status_prev_time;
    ros::Time _start_time;

    bool _is_waiting_on_result;
    std::condition_variable _result_get_condition;
    PacketResult* _result_get;

    // Publishers
    ros::Publisher _ping_pub;
    ros::Publisher _packet_count_pub;
    ros::Publisher _packet_rate_pub;

    bool reOpenDevice();
    bool openDevice();
    void closeDevice();
    bool didDeviceTimeout();

    void checkHandshakes();
    void compareHandshakes(PacketResult* result);

    void pollDeviceTask();
    void writeDeviceTask();
    bool pollDevice();

protected:
    ros::NodeHandle nh;  // ROS node handle

    std::mutex _result_get_lock;

    void begin();

    void writePacket(string category, const char *formats, ...);
    void writeHandshakePacket(string category, const char *formats, double write_interval, double timeout, ...);
    PacketResult* getResult(string category, const char *formats, double write_interval, double timeout, ...);

    virtual void packetCallback(PacketResult* result);
    void pingCallback(const ros::TimerEvent& event);
    double getLocalTime();

    void publishStatusMessages(float ping);

    virtual void writeDeviceTick();

public:
    BwSerialTunnel(ros::NodeHandle* nodehandle);
    void join();
};
