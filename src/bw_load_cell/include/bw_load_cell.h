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
#include "bw_interfaces/CalibrateScale.h"
#include "std_srvs/Trigger.h"

#include "tunnel_protocol.h"
#include "bw_serial_tunnel.h"


class BwLoadCell : public BwSerialTunnel {
private:
    // Parameters
    double _calibration_value;

    // Members
    bool _is_calibrated;
    double _last_mass;

    // Publishers
    ros::Publisher _load_cell_pub;

    // Service Servers
    ros::ServiceServer _tare_srv;
    ros::ServiceServer _calibrate_srv;
    ros::ServiceServer _reset_srv;

    void packetCallback(PacketResult* result);
    bool writeCalibration(double calibration_value);
    bool calibrateCallback(bw_interfaces::CalibrateScale::Request &req, bw_interfaces::CalibrateScale::Response &resp);
    bool tareCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool resetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
public:
    BwLoadCell(ros::NodeHandle* nodehandle);
    int run();
};