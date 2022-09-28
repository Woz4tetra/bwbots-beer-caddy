
#include <Arduino.h>

void setup()
{
    Serial.begin(9600);
    Serial2.begin(9600);
}

void loop()
{
    Serial.println("something");
    Serial2.println("something");
    delay(500);
}
