#include <Arduino.h>
#include "tunnel/serial.h"
#include "chassis.h"


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


void setup()
{
    // Start serial tunnel client
    tunnel_begin();
    chassis.begin();
    DEBUG_SERIAL.begin(DEBUG_BAUD);

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
        int32_t state;
        if (!result->getInt(state)) { PROTOCOL_SERIAL.println(F("Failed motor_enable")); return; }
        chassis.set_motor_enable((bool)state);
    }
    else if (category.equals("is_motor_enabled")) {
        int32_t state = (int32_t)(chassis.get_motor_enable());
        tunnel_writePacket("is_motor_enabled", "d", state);
    }
    else if (category.equals("l")) {
        int32_t velocity;
        if (!result->getInt(velocity)) { return; }
        chassis.set_left_motor(velocity);
    }
    else if (category.equals("r")) {
        int32_t velocity;
        if (!result->getInt(velocity)) { return; }
        chassis.set_right_motor(velocity);
    }
}

void loop()
{
    packetCallback(tunnel_readPacket());
    if (chassis.update()) {
        tunnel_writePacket("enc", "ddff", 
            chassis.get_left_encoder(), chassis.get_right_encoder(),
            chassis.get_left_speed(), chassis.get_right_speed()
        );
    }
}
