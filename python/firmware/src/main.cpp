#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_PWMServoDriver.h>

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
    32,  // motorr_dr1
    31,  // motorr_dr2
    30,  // motorr_pwm
    23,  // motorl_enca
    22,  // motorl_encb
    21,  // motorr_enca
    20  // motorr_encb
);

const int BNO055_RST_PIN = 25;
Adafruit_BNO055 bno(-1, BNO055_ADDRESS_A, &I2C_BUS_2);

BalanceController balance_controller(&chassis, &bno, 2.65, 4.0, 0.05, 0.0);

const int SERVOMIN = 150; // This is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 600; // This is the 'maximum' pulse length count (out of 4096)
const int USMIN = 600; // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
const int USMAX = 2400; // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
const int SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40, I2C_BUS_2);

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

    servos.begin();
    servos.setOscillatorFrequency(27000000);
    servos.setPWMFreq(SERVO_FREQ);

    delay(10);
}

void setup()
{
    // Start serial tunnel client
    tunnel_begin();
    DEBUG_SERIAL.begin(DEBUG_BAUD);

    chassis.begin();
    setup_i2c();
    setup_bno();
    balance_controller.reset();

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
        tunnel_writePacket("is_motor_enabled", "b", state);
        balance_controller.set_enable(false);
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
        double setpoint, kp, kd;
        if (!result->getDouble(setpoint))  { PROTOCOL_SERIAL.println(F("Failed to set setpoint")); return; }
        if (!result->getDouble(kp))  { PROTOCOL_SERIAL.println(F("Failed to set kp")); return; }
        if (!result->getDouble(kd))  { PROTOCOL_SERIAL.println(F("Failed to set kd")); return; }
        balance_controller.set_angle_setpoint(setpoint);
        balance_controller.set_kp(kp);
        balance_controller.set_kd(kd);
        tunnel_writePacket("balance", "eee", 
            balance_controller.get_angle_setpoint(),
            balance_controller.get_kp(),
            balance_controller.get_kd()
        );
    }
    else if (category.equals("balen")) {
        bool enable;
        if (!result->getBool(enable))  { PROTOCOL_SERIAL.println(F("Failed to set enable")); return; }
        balance_controller.set_enable(enable);
        tunnel_writePacket("balen", "b", 
            balance_controller.get_enable()
        );
    }
    else if (category.equals("s")) {
        uint16_t pulselen;
        uint8_t servonum;
        if (!result->getUInt8(servonum)) { PROTOCOL_SERIAL.println(F("Failed to get servo number")); return; }
        if (!result->getUInt16(pulselen)) { PROTOCOL_SERIAL.println(F("Failed to get servo pulse length")); return; }
        if (0 <= servonum && servonum < 16) {
            servos.setPWM(servonum, 0, pulselen);
        }
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
        tunnel_writePacket("motors", "dd", 
            chassis.get_left_command(), chassis.get_right_command()
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
