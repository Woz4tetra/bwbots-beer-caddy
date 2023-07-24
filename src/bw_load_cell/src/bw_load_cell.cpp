#include "bw_load_cell.h"

BwLoadCell::BwLoadCell(ros::NodeHandle* nodehandle) :
    BwSerialTunnel(nodehandle)
{
    ros::param::param<double>("~calibration_value", _calibration_value, 0.0);

    _is_calibrated = false;
    _last_mass = 0.0;

    _load_cell_pub = nh.advertise<bw_interfaces::LoadCell>("mass", 50);

    _tare_srv = nh.advertiseService("tare", &BwLoadCell::tareCallback, this);
    _calibrate_srv = nh.advertiseService("calibrate", &BwLoadCell::calibrateCallback, this);
    _reset_srv = nh.advertiseService("reset", &BwLoadCell::resetCallback, this);

    begin();

    ROS_INFO("bw_load_cell init complete");
}

bool BwLoadCell::writeCalibration(double calibration_value)
{
    if (calibration_value == 0.0) {
        ROS_WARN("Invalid calibration value! %f", calibration_value);
        return false;
    }
    else {
        ROS_INFO("Writing calibration value: %f", calibration_value);
        writePacket("calibrate", "f", (float)calibration_value);
        _is_calibrated = true;
        return true;
    }
}

bool BwLoadCell::tareCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
    writePacket("tare", "");
    resp.success = true;
    return true;
}

bool BwLoadCell::calibrateCallback(bw_interfaces::CalibrateScale::Request &req, bw_interfaces::CalibrateScale::Response &resp)
{
    _calibration_value = _last_mass / req.mass;
    bool result = writeCalibration(_calibration_value);
    resp.calibration_value = _calibration_value;
    return result;
}

bool BwLoadCell::resetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
    resp.success = writeCalibration(1.0);
    _calibration_value = 0.0;
    _is_calibrated = false;
    return resp.success;
}

void BwLoadCell::packetCallback(PacketResult* result)
{
    BwSerialTunnel::packetCallback(result);
    string category = result->getCategory();
    if (category.compare("g") == 0) {
        float load_cell_value;
        if (!result->getFloat(load_cell_value))  { ROS_ERROR("Failed to get load_cell_value"); return; }

        _last_mass = (double)load_cell_value;

        if (_is_calibrated) {
            bw_interfaces::LoadCell lc_msg;
            lc_msg.mass = (double)load_cell_value;
            _load_cell_pub.publish(lc_msg);
        }
        else {
            ROS_DEBUG("Load cell isn't calibrated! Not publishing.");
            if (_calibration_value != 0.0 && _last_mass != 0.0) {
                writeCalibration(_calibration_value);
            }
        }
    }
}

int BwLoadCell::run()
{
    ros::spin();
    this->join();
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
