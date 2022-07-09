#include <Arduino.h>
#include <tunnel/serial.h>


void setup()
{
    // Start serial tunnel client
    tunnel_begin();
    DEBUG_SERIAL.begin(DEBUG_BAUD);
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
}

void loop()
{
    // uint32_t current_time = millis();
    packetCallback(tunnel_readPacket());
}
