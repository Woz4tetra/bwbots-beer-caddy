#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

#ifndef WIFI_SSID
#error WIFI_SSID is not defined
#endif

#ifndef WIFI_PASSWORD
#error WIFI_PASSWORD is not defined
#endif

#ifndef MQTT_SERVER
#error MQTT_SERVER is not defined
#endif

#define XSTR(x) STR(x)
#define STR(x) #x

#pragma message "ssid: " XSTR(WIFI_SSID)
#pragma message "mqtt server: " XSTR(MQTT_SERVER)

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* mqtt_server = MQTT_SERVER;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

const int BREAK_BEAM_PIN = 15;
const int BUTTON_PIN = 14;
const int LIMIT_SWITCH_PIN = 21;
const int RELAY_PIN = 12;


void setup_wifi() {
    
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    int blink = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        digitalWrite(LED_BUILTIN, blink % 2 == 0);
        Serial.print(".");
        blink++;
    }
    for (blink = 0; blink < 16; blink++) {
        digitalWrite(LED_BUILTIN, blink % 2 == 0);
        delay(50);
    }
    digitalWrite(LED_BUILTIN, LOW);

    randomSeed(micros());

    Serial.println("");
    Serial.println("WiFi connected");  
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '1') {
        digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
        // but actually the LED is on; this is because
        // it is active low on the ESP-01)
    }
    else {
        digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
    }
}

void reconnect() {
  // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (client.connect(clientId.c_str())) {
            Serial.println("connected");
            // Once connected, publish an announcement...
            client.publish("outTopic", "hello world");
            // ... and resubscribe
            client.subscribe("inTopic");
            digitalWrite(LED_BUILTIN, LOW);
        }
        else {
            for (unsigned int blink = 0; blink < 6; blink++) {
                digitalWrite(LED_BUILTIN, blink % 2 == 0);
                delay(50);
            }
            digitalWrite(LED_BUILTIN, LOW);
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 1 second");
            // Wait 1 second before retrying
            delay(1000);
        }
    }
}

bool IS_DISPENSING = false;

void start_dispense() {
    IS_DISPENSING = true;
    digitalWrite(RELAY_PIN, HIGH);
}

void stop_dispense() {
    IS_DISPENSING = false;
    digitalWrite(RELAY_PIN, LOW);
}

void check_dispense() {
    if (!IS_DISPENSING) {
        return;
    }
    if (digitalRead(LIMIT_SWITCH_PIN)) {
        stop_dispense();
    }
}

bool button_state = false;
bool prev_button_read_state = false;
uint32_t last_debounce_time = 0;
uint32_t DEBOUNCE_DELAY = 50;

bool read_button() {
    return !digitalRead(BUTTON_PIN);
}

bool get_button_state() {
    uint32_t current_time = millis();
    bool state = read_button();
    if (state != prev_button_read_state) {
        last_debounce_time = current_time;
    }
    prev_button_read_state = state;

    if (current_time - last_debounce_time > DEBOUNCE_DELAY) {
        button_state = state;
    }
    return button_state;
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BREAK_BEAM_PIN, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(LIMIT_SWITCH_PIN, INPUT);

    Serial.begin(115200);
    delay(100);

    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        switch (c)
        {
        case 's':
            start_dispense();
            break;
        default:
            break;
        }
    }
    if (get_button_state()) {
        start_dispense();
    }
    check_dispense();

    if (!client.connected() && !IS_DISPENSING) {
        reconnect();
    }
    client.loop();

    unsigned long now = millis();
    if (now - lastMsg > 2000) {
        lastMsg = now;
        ++value;
        snprintf(msg, MSG_BUFFER_SIZE, "hello world #%d", value);
        Serial.print("Publish message: ");
        Serial.println(msg);
        client.publish("outTopic", msg);
    }
}
