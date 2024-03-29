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
#include <memory>
#include <stdexcept>

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
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "bw_interfaces/BwDriveModule.h"
#include "bw_interfaces/BwDriveState.h"
#include "bw_interfaces/OdomReset.h"
#include "bw_interfaces/ChargeState.h"
#include "bw_interfaces/PlaySequence.h"
#include "bw_interfaces/StopSequence.h"
#include "bw_interfaces/BwSequence.h"
#include "bw_interfaces/BwSequenceState.h"

#include "tunnel_protocol.h"
#include "bw_serial_tunnel.h"

using namespace std;

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

class BwTunnel : public BwSerialTunnel {
private:
    // Parameters
    bool _publish_odom_tf;
    string _base_frame;
    string _odom_frame;
    string _map_frame;

    double _cmd_vel_timeout_param;
    ros::Duration _cmd_vel_timeout;

    std::vector<std::string> _joint_names;
    int _num_modules;

    float _voltage_delta_warning;
    float _voltage_minimum_warning;
    ros::Duration _min_terminal_log_interval;

    // Members

    // Messages
    nav_msgs::Odometry _odom_msg;
    vector<std_msgs::Float64*>* _raw_joint_msgs;

    ros::Time _prev_twist_timestamp;
    double _twist_cmd_vx, _twist_cmd_vy, _twist_cmd_vt;

    ros::Time _largest_voltage_time;
    float _largest_voltage;

    ros::Time _prev_log_time;
    float _prev_log_voltage;

    bw_interfaces::BwDriveState _module_msg;

    // Publishers
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    ros::Publisher _odom_pub;
    vector<ros::Publisher>* _raw_joint_pubs;
    ros::Publisher _charge_pub;
    ros::Publisher _button_pub;
    ros::Publisher _button_counter_pub;
    ros::Publisher _button_counter_result_pub;
    ros::Publisher _is_enabled_pub;
    ros::Publisher _module_pub;
    ros::Publisher _sequence_state_pub;

    // Subscribers
    ros::Subscriber _twist_sub;
    ros::Subscriber _set_enabled_sub;
    tf::TransformListener _tf_listener;
    ros::Subscriber _module_sub;
    ros::Subscriber _sequence_sub;

    // Service Servers
    ros::ServiceServer _odom_reset_srv;
    ros::ServiceServer _play_sequence_srv;
    ros::ServiceServer _stop_sequence_srv;

    void addJointPub(string name);
    void logCharger(float voltage);
    void warnAllTerminals(string message);

    // Service callbacks
    bool odomResetCallback(bw_interfaces::OdomReset::Request &req, bw_interfaces::OdomReset::Response &resp);
    bool playSequenceCallback(bw_interfaces::PlaySequence::Request &req, bw_interfaces::PlaySequence::Response &resp);
    bool stopSequenceCallback(bw_interfaces::StopSequence::Request &req, bw_interfaces::StopSequence::Response &resp);

    // Write tasks
    void publishCmdVel();

    // Publish tasks
    void publishOdom(ros::Time recv_time, double x, double y, double t, double vx, double vy, double vt);
    void publishJoint(ros::Time recv_time, int joint_index, double joint_position);

    // Topic callbacks
    void twistCallback(const geometry_msgs::TwistConstPtr& msg);
    void setEnabledCallback(const std_msgs::BoolConstPtr& msg);
    void moduleCommandCallback(const bw_interfaces::BwDriveModuleConstPtr& msg);
    void loadSequenceCallback(const bw_interfaces::BwSequenceConstPtr& msg);

    // Overridden methods
    virtual void packetCallback(PacketResult* result);
    virtual void writeDeviceTick();

public:
    BwTunnel(ros::NodeHandle* nodehandle);
    int run();
};
