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

#ifndef DEVICE_NAME
#error DEVICE_NAME is not defined
#endif

#define XSTR(x) STR(x)
#define STR(x) #x

#pragma message "ssid: " XSTR(WIFI_SSID)
#pragma message "mqtt server: " XSTR(MQTT_SERVER)
#pragma message "device name: " XSTR(DEVICE_NAME)

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* mqtt_server = MQTT_SERVER;
const char* device_name = DEVICE_NAME;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long prev_publish_time = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

const int BREAK_BEAM_PIN = 15;
const int BUTTON_PIN = 14;
const int LIMIT_SWITCH_PIN = 21;
const int RELAY_PIN = 12;


void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void start_dispense();
void stop_dispense();
void check_dispense();
bool has_drink();
bool read_button();
bool get_button_state();
void publish_dispense_status(bool is_dispensing);
void publish_dispense_done(bool success);
void publish_drink_status(bool has_drink);
void reconnect();


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

    if (strcmp(topic, "start_dispense") == 0) {
        memcpy(msg, payload, length + 1);
        msg[length] = 0;
        if (strcmp(msg, device_name) == 0) {
            Serial.println("Starting dispense");
            start_dispense();
        }
    }
}

bool is_dispensing = false;
uint32_t dispense_start_time = 0;
const uint32_t min_dispense_time = 100;
const uint32_t max_dispense_time = 5000;


void start_dispense() {
    is_dispensing = true;
    digitalWrite(RELAY_PIN, HIGH);
    dispense_start_time = millis();
}

void stop_dispense() {
    is_dispensing = false;
    digitalWrite(RELAY_PIN, LOW);
}

void check_dispense() {
    if (!is_dispensing) {
        return;
    }
    uint32_t current_time = millis();
    if (current_time - dispense_start_time > min_dispense_time && digitalRead(LIMIT_SWITCH_PIN)) {
        stop_dispense();
        publish_dispense_done(true);
    }
    if (current_time - dispense_start_time > max_dispense_time) {
        stop_dispense();
        publish_dispense_done(false);
    }
}

bool has_drink() {
    return !digitalRead(BREAK_BEAM_PIN);
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


void publish_dispense_status(bool is_dispensing) {
    snprintf(msg, MSG_BUFFER_SIZE, "%s\t%d", device_name, is_dispensing);
    Serial.print("Publish dispense status: ");
    Serial.println(msg);
    client.publish("is_dispensing", msg);
}

void publish_dispense_done(bool success) {
    snprintf(msg, MSG_BUFFER_SIZE, "%s\t%d", device_name, success);
    Serial.print("Publish dispense done: ");
    Serial.println(msg);
    client.publish("dispense_done", msg);
}

void publish_drink_status(bool has_drink) {
    snprintf(msg, MSG_BUFFER_SIZE, "%s\t%d", device_name, has_drink);
    Serial.print("Publish drink status: ");
    Serial.println(msg);
    client.publish("has_drink", msg);
}

void reconnect() {
  // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection on ");
        Serial.print(mqtt_server);
        Serial.print("...");
        // Create a random client ID
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (client.connect(clientId.c_str())) {
            Serial.println("connected");
            client.subscribe("start_dispense");
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

    if (!client.connected() && !is_dispensing) {
        reconnect();
    }
    client.loop();

    unsigned long now = millis();
    if (now - prev_publish_time > 1000) {
        prev_publish_time = now;
        publish_dispense_status(is_dispensing);
        publish_drink_status(has_drink());
    }
}
