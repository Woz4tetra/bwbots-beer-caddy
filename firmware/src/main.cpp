#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

#include "tunnel/serial.h"
#include "chassis.h"
#include "balance_controller.h"


#define I2C_BUS_1 Wire
#define I2C_BUS_2 Wire1

Chassis chassis(
    26,  // motor_stby
    27,  // motorl_dr1
    28,  // motorl_dr2
    29,  // motorl_pwm
    31,  // motorr_dr1
    32,  // motorr_dr2
    30,  // motorr_pwm
    23,  // motorl_enca
    22,  // motorl_encb
    21,  // motorr_enca
    20  // motorr_encb
);

const int BNO055_RST_PIN = 25;
Adafruit_BNO055 bno(-1, BNO055_ADDRESS_A, &I2C_BUS_2);

BalanceController balance_controller(&chassis, &bno);

void setup_i2c()
{
    I2C_BUS_1.begin();
    I2C_BUS_1.setSDA(18);
    I2C_BUS_1.setSCL(19);
    I2C_BUS_2.begin();
    I2C_BUS_2.setSDA(37);
    I2C_BUS_2.setSCL(38);
    DEBUG_SERIAL.println("I2C initialized.");
}

void setup_bno()
{
    pinMode(BNO055_RST_PIN, OUTPUT);
    digitalWrite(BNO055_RST_PIN, HIGH);
    delay(800);
    if (!bno.begin()) {
        DEBUG_SERIAL.println("No BNO055 detected!! Check your wiring or I2C address");
        return;
    }
    delay(100);
    bno.setExtCrystalUse(true);
    delay(100);
}

void setup()
{
    // Start serial tunnel client
    tunnel_begin();
    DEBUG_SERIAL.begin(DEBUG_BAUD);

    chassis.begin();
    setup_i2c();
    setup_bno();
    balance_controller.begin();

    chassis.set_speed_smooth_left_k(0.9);
    chassis.set_speed_smooth_right_k(0.9);
}

void packetCallback(PacketResult* result)
{
    // if the result is not set for some reason, don't do anything
    if (result == NULL) {
        return;
    }

    // Extract category and check which event it maps to
    String category = result->getCategory();
    if (category.equals("ping")) {
        // Respond to ping by writing back the same value
        float value;
        if (!result->getFloat(value)) { return; }
        tunnel_writePacket("ping", "f", value);
    }
    else if (category.equals("motor_enable")) {
        uint8_t state;
        if (!result->getUInt8(state)) { PROTOCOL_SERIAL.println(F("Failed motor_enable")); return; }
        chassis.set_motor_enable((bool)state);
    }
    else if (category.equals("is_motor_enabled")) {
        uint8_t state = (uint8_t)(chassis.get_motor_enable());
        tunnel_writePacket("is_motor_enabled", "j", state);
    }
    else if (category.equals("l")) {
        int32_t velocity;
        if (!result->getInt32(velocity)) { return; }
        chassis.set_left_motor(velocity);
    }
    else if (category.equals("r")) {
        int32_t velocity;
        if (!result->getInt32(velocity)) { return; }
        chassis.set_right_motor(velocity);
    }
    else if (category.equals("balance")) {
        float setpoint;
        if (!result->getFloat(setpoint))  { PROTOCOL_SERIAL.println(F("Failed to set setpoint")); return; }
        balance_controller.set_angle_setpoint(setpoint);
        tunnel_writePacket("balance", "f", 
            balance_controller.get_angle_setpoint()
        );
    }
}

void loop()
{
    packetCallback(tunnel_readPacket());
    if (chassis.update()) {
        int32_t left = (int32_t)chassis.get_left_encoder();
        int32_t right = (int32_t)chassis.get_right_encoder();
        tunnel_writePacket("enc", "ddff", 
            left, right,
            chassis.get_left_speed(), chassis.get_right_speed()
        );
    }
    if (balance_controller.update()) {
        tunnel_writePacket("imu", "fff", 
            balance_controller.get_imu_x(),
            balance_controller.get_imu_y(),
            balance_controller.get_imu_z()
        );
    }
}
