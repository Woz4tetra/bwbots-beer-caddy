#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "tunnel/serial.h"

#define I2C_BUS_1 Wire
// #define I2C_BUS_2 Wire1

const int STATUS_LED = 32;
const int MOTOR_ENABLE = 33;
const int USB_SENSE = 34;
const int BUTTON_LED = 14;
const int BUILTIN_LED = 13;


const int SERVOMIN = 150; // This is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 600; // This is the 'maximum' pulse length count (out of 4096)
const int USMIN = 600; // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
const int USMAX = 2400; // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
const int SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40, I2C_BUS_1);

void setup_i2c()
{
    I2C_BUS_1.begin();
    I2C_BUS_1.setSDA(18);
    I2C_BUS_1.setSCL(19);
    // I2C_BUS_2.begin();
    // I2C_BUS_2.setSDA(37);
    // I2C_BUS_2.setSCL(38);
    DEBUG_SERIAL.println("I2C initialized.");
}

void setup_bno()
{
    servos.begin();
    servos.setOscillatorFrequency(27000000);
    servos.setPWMFreq(SERVO_FREQ);

    pinMode(BUTTON_LED, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);
    pinMode(BUILTIN_LED, OUTPUT);

    for (int count = 0; count < 10; count++) {
        digitalWrite(BUILTIN_LED, LOW);
        digitalWrite(BUTTON_LED, LOW);
        delay(10);
        digitalWrite(BUILTIN_LED, HIGH);
        digitalWrite(BUTTON_LED, HIGH);
        delay(10);
    }

    delay(10);
}

void setup()
{
    // Start serial tunnel client
    tunnel_begin();
    DEBUG_SERIAL.begin(DEBUG_BAUD);

    setup_i2c();
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
}
