#pragma once

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include "chassis.h"

class BalanceController
{
public:
    BalanceController(Chassis* chassis, Adafruit_BNO055* bno);

    void begin();
    bool update();

    float get_imu_x()  { return orientationData.orientation.x; }
    float get_imu_y()  { return orientationData.orientation.y; }
    float get_imu_z()  { return orientationData.orientation.z; }

    float get_angle_setpoint()  { return angle_setpoint; }
    void set_angle_setpoint(float setpoint)  { angle_setpoint = setpoint; }

private:
    Adafruit_BNO055* bno;
    Chassis* chassis;
    sensors_event_t orientationData;
    uint32_t report_timer, sample_delay;
    float angle_setpoint;
};
