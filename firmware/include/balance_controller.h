#pragma once

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include "chassis.h"
#include "pd_controller.h"

class BalanceController
{
public:
    BalanceController(Chassis* chassis, Adafruit_BNO055* bno, double setpoint, double kp, double kd, double tolerance);

    void reset();
    bool update();

    double get_imu_x()  { return orientationData.orientation.x * M_PI / 180.0; }
    double get_imu_y()  { return orientationData.orientation.y * M_PI / 180.0; }
    double get_imu_z()  { return orientationData.orientation.z * M_PI / 180.0; }

    double get_angle_setpoint()  { return angle_setpoint; }
    void set_angle_setpoint(double setpoint)  { angle_setpoint = setpoint; }

    void set_enable(bool state)  { enabled = state; }
    bool get_enable()  { return enabled; }

    void set_kp(double kp)  { this->controller->set_kp(kp); }
    double get_kp()  { return this->controller->get_kp(); }

    void set_kd(double kd)  { this->controller->set_kd(kd); }
    double get_kd()  { return this->controller->get_kd(); }

private:
    Adafruit_BNO055* bno;
    Chassis* chassis;
    PDController* controller;
    sensors_event_t orientationData;
    uint32_t report_timer, sample_delay;
    double angle_setpoint;
    bool enabled;
    const double VOLTS_TO_PWM = 255.0 / 6.0;
};
