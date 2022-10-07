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
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "ros/console.h"
#include "tf/transform_listener.h"
#include "serial/serial.h"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "bw_interfaces/BwDriveModule.h"
#include "bw_interfaces/OdomReset.h"
#include "bw_interfaces/ChargeState.h"

#include "tunnel_protocol.h"

using namespace std;

int sign_of(double x) {
    return (x > 0) - (x < 0);
}


class BwTunnel {
private:
    ros::NodeHandle nh;  // ROS node handle

    // Parameters
    string _device_path;
    int _device_baud;
    int _open_attempts;

    double _tunnel_rate;  // Hz
    double _write_rate;  // Hz

    bool _publish_odom_tf;
    string _base_frame;
    string _odom_frame;
    string _map_frame;

    double _cmd_vel_timeout_param;
    ros::Duration _cmd_vel_timeout;

    std::vector<std::string> _joint_names;

    // Members
    const int READ_BUFFER_LEN = 4096;
    char* _read_buffer;
    bool _initialized;

    int _unparsed_index;
    char* _write_buffer;

    std::mutex _write_lock;
    boost::thread* _poll_thread;
    boost::thread* _write_thread;
    TunnelProtocol* _protocol;
    serial::Serial _device;

    ros::Time _last_read_time;
    ros::Duration _last_read_threshold;

    ros::Timer _ping_timer;
    ros::Time _prev_ping_time;
    ros::Duration _ping_interval;

    int _packet_count;
    int _status_prev_count;
    ros::Time _status_prev_time;
    ros::Time _start_time;

    // Messages
    nav_msgs::Odometry _odom_msg;
    vector<std_msgs::Float64*>* _raw_joint_msgs;

    ros::Time _prev_twist_timestamp;
    double _twist_cmd_vx, _twist_cmd_vy, _twist_cmd_vt;

    // Publishers
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    ros::Publisher _odom_pub;
    ros::Publisher _ping_pub;
    vector<ros::Publisher>* _raw_joint_pubs;
    ros::Publisher _packet_count_pub;
    ros::Publisher _packet_rate_pub;
    ros::Publisher _charge_pub;
    ros::Publisher _button_pub;
    ros::Publisher _is_enabled_pub;
    ros::Publisher _module_pub;

    // Subscribers
    ros::Subscriber _twist_sub;
    ros::Subscriber _set_enabled_sub;
    tf::TransformListener _tf_listener;
    ros::Subscriber _module_sub;

    // Service Servers
    ros::ServiceServer _odom_reset_srv;

    bool reOpenDevice();
    bool openDevice();
    void closeDevice();
    bool didDeviceTimeout();

    void writePacket(string category, const char *formats, ...);

    void packetCallback(PacketResult* result);
    void pingCallback(const ros::TimerEvent& event);
    double getLocalTime();

    void addJointPub(string name);

    // Service callbacks
    bool odom_reset_callback(bw_interfaces::OdomReset::Request &req, bw_interfaces::OdomReset::Response &resp);

    void publishCmdVel();

    void publishOdom(ros::Time recv_time, double x, double y, double t, double vx, double vy, double vt);
    void publishJoint(ros::Time recv_time, int joint_index, double joint_position);
    void publishStatusMessages(float ping);

    void twistCallback(const geometry_msgs::TwistConstPtr& msg);
    void setEnabledCallback(const std_msgs::BoolConstPtr& msg);
    void moduleCommandCallback(const bw_interfaces::BwDriveModuleConstPtr& msg);

    void pollDeviceTask();
    void writeDeviceTask();
    bool pollDevice();

public:
    BwTunnel(ros::NodeHandle* nodehandle);
    int run();
};
