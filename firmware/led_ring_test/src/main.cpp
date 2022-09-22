#include <Adafruit_NeoPixel.h>


const int LED_RING = 39;
const int NUM_PIXELS = 24;
Adafruit_NeoPixel led_ring(NUM_PIXELS, LED_RING, NEO_GRBW + NEO_KHZ800);


void setup()
{
    led_ring.begin();
}

void loop()
{
    for(int i = 0; i < NUM_PIXELS; i++) {
        led_ring.setPixelColor(i, led_ring.Color(0, 0, 150));
        led_ring.show();
        delay(10);
    }

    delay(500);
    for(int i = 0; i < NUM_PIXELS; i++) {
        led_ring.setPixelColor(i, led_ring.Color(0, 0, 0));
        led_ring.show();
    }
    delay(500);
}