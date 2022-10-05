#include "HX711.h"
#include "tunnel_serial.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

HX711 scale;

long reading;
uint32_t read_timer = 0;
const uint32_t READ_INTERVAL_MS = 1000;

#define PROTOCOL_SERIAL Serial
#define PROTOCOL_BAUD 115200

TunnelSerial* tunnel;


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
        tunnel->writePacket("ping", "f", value);
    }
}

void setup() {
    PROTOCOL_SERIAL.begin(PROTOCOL_BAUD);
    tunnel = new TunnelSerial(NULL, &PROTOCOL_SERIAL);
    
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
}

void loop()
{
    packetCallback(tunnel->readPacket());
    uint32_t current_time = millis();
    if (current_time - read_timer > READ_INTERVAL_MS) {
        read_timer = current_time;
        bool is_ready = scale.is_ready();
        if (is_ready) {
            reading = scale.read();
        }
        tunnel->writePacket("g", "bd", is_ready, reading);
    }
}
