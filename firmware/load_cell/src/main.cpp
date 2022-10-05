#include "HX711.h"
#include "tunnel_serial.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

HX711 scale;

float reading;
int average_window = 5;
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
    else if (category.equals("tare")) {
        float calibration;
        if (!result->getFloat(calibration)) { return; }
        scale.set_scale(calibration);
        scale.tare();
    }
    else if (category.equals("window")) {
        if (!result->getInt16(average_window)) { return; }
    }
}

void setup() {
    PROTOCOL_SERIAL.begin(PROTOCOL_BAUD);
    tunnel = new TunnelSerial(NULL, &PROTOCOL_SERIAL);
    
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale.set_scale();
    scale.tare();
}

void loop()
{
    packetCallback(tunnel->readPacket());
    uint32_t current_time = millis();
    if (current_time - read_timer > READ_INTERVAL_MS) {
        if (scale.wait_ready_timeout(1000)) {
            read_timer = current_time;
            // reading = (float)scale.read();
            reading = scale.get_units(average_window);
            tunnel->writePacket("g", "f", reading);
        }
    }
}
