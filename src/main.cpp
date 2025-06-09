#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include "debug.h"
#include <Preferences.h>

// Pins for SN74HC595 shift registers
const uint8_t DATA_PIN = 14;
const uint8_t CLOCK_PIN = 13;
const uint8_t LATCH_PIN = 12;
const uint8_t OE_PIN = 5; // active low

// internal state of shift register (active high relays)
static uint16_t shiftState = 0x0000; // all off (LOW)

// desired zone states (true=open, false=closed)
static bool zoneState[NUM_ZONES] = {false};

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

Preferences prefs;

const unsigned long SAVE_INTERVAL_MS = 30000;
unsigned long lastChangeTime = 0;
bool stateChanged = false;

void writeShiftRegister() {
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, (shiftState >> 8) & 0xFF);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, shiftState & 0xFF);
    digitalWrite(LATCH_PIN, HIGH);
}

void setRelay(uint8_t index, bool active) {
    if (active)
        shiftState |= (1 << index); // active high
    else
        shiftState &= ~(1 << index);
}

bool coilOnForOpen() { return COIL_ON_FOR_OPEN != 0; }

bool coilStateForZone(bool open) {
    return open ? coilOnForOpen() : !coilOnForOpen();
}

uint16_t zoneStateToBits() {
    uint16_t val = 0;
    for (uint8_t i = 0; i < NUM_ZONES; ++i) {
        if (zoneState[i])
            val |= (1 << i);
    }
    return val;
}

void bitsToZoneState(uint16_t bits) {
    for (uint8_t i = 0; i < NUM_ZONES; ++i) {
        zoneState[i] = bits & (1 << i);
    }
}


void printZoneState() {
    DEBUG_PRINT("Zones state: ");
    for (uint8_t i = 0; i < NUM_ZONES; ++i) {
        DEBUG_PRINT(zoneState[i] ? "1" : "0");
    }
    DEBUG_PRINTLN("");
}


void loadState() {
    uint16_t bits = prefs.getUShort("state", 0);
    bitsToZoneState(bits);
    DEBUG_PRINT("Loaded state: ");
    printZoneState();
}

void saveState() {
    uint16_t bits = zoneStateToBits();
    prefs.putUShort("state", bits);
    DEBUG_PRINT("Saved state: ");
    printZoneState();
}

void publishZoneState(uint8_t zone) {
    char topic[64];
    snprintf(topic, sizeof(topic), "%s/zone%u/state", BASE_TOPIC, zone + 1);
    mqttClient.publish(topic, zoneState[zone] ? "ON" : "OFF", true);

//    DEBUG_PRINT("sending zoneState:");
//    DEBUG_PRINT(zone + 1);
//    DEBUG_PRINT("=");
//    DEBUG_PRINTLN(zoneState[zone] ? "ON" : "OFF");

}

void publishAllStates() {
    for (uint8_t i = 0; i < NUM_ZONES; ++i) {
        publishZoneState(i);
    }
}


void applyZones() {
    DEBUG_PRINT("Applying zones: ");
    for (uint8_t i = 0; i < NUM_ZONES; ++i) {
        DEBUG_PRINT(zoneState[i] ? "1" : "0");
#if ACTUATE_RELAYS
        setRelay(i, coilStateForZone(zoneState[i]));
#endif
    }
    DEBUG_PRINTLN("");
    
    publishAllStates();

#if ACTUATE_RELAYS
    writeShiftRegister();

    // master relay on
    setRelay(MASTER_RELAY_INDEX, true);
    writeShiftRegister();
    delay(ZONE_PULSE_MS);

    // master relay off
    setRelay(MASTER_RELAY_INDEX, false);
    writeShiftRegister();

    // disable zone relays
    for (uint8_t i = 0; i < NUM_ZONES; ++i) {
        setRelay(i, false);
    }
    writeShiftRegister();
#else
    DEBUG_PRINTLN(" (dry run - relays not actuated)");
#endif

   // publishAllStates();
}

void sendDiscovery() {
    DEBUG_PRINT("Sending discovery messages...");
    for (uint8_t i = 0; i < NUM_ZONES; ++i) {
        char topic[128];
        snprintf(topic, sizeof(topic),
                 "homeassistant/switch/%s/zone%u/config", DEVICE_NAME, i + 1);

        char payload[256];
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Zone %u\",\"command_topic\":\"%s/zone%u/set\"," \
                 "\"state_topic\":\"%s/zone%u/state\",\"uniq_id\":\"%s_zone%u\"," \
                 "\"payload_on\":\"ON\",\"payload_off\":\"OFF\"}",
                 i + 1, BASE_TOPIC, i + 1, BASE_TOPIC, i + 1,
                 DEVICE_NAME, i + 1);
        DEBUG_PRINT("sending payload: ");
        DEBUG_PRINTLN(payload);
        mqttClient.publish(topic, payload, true);
    }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
    String msg;
    for (unsigned int i = 0; i < length; ++i) {
        msg += (char)payload[i];
    }
    DEBUG_PRINT("Message received [");
    DEBUG_PRINT(topic);
    DEBUG_PRINT("] ");
    DEBUG_PRINTLN(msg);
    String t(topic);
    String prefix = String(BASE_TOPIC) + "/zone";
    if (t.startsWith(prefix) && t.endsWith("/set")) {
        int zone = t.substring(prefix.length(), t.length() - 4).toInt();
        if (zone >= 1 && zone <= NUM_ZONES) {
            bool newState = msg.equalsIgnoreCase("ON") || msg.equalsIgnoreCase("OPEN");
            zoneState[zone - 1] = newState;
            stateChanged = true;
            lastChangeTime = millis();
            applyZones();
        }
    }
}

void reconnectMqtt() {
    while (!mqttClient.connected()) {
        DEBUG_PRINT("Attempting MQTT connection...");
        if (mqttClient.connect(DEVICE_NAME, MQTT_USER, MQTT_PASSWORD)) {
            DEBUG_PRINTLN("connected");
            String sub = String(BASE_TOPIC) + "/+/set";
            mqttClient.subscribe(sub.c_str());
            sendDiscovery();
            publishAllStates();
        } else {
            DEBUG_PRINT("failed, rc=");
            DEBUG_PRINTLN(mqttClient.state());
            delay(5000);
        }
    }
}

void connectWifi() {
    DEBUG_PRINT("Connecting to WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        DEBUG_PRINT(".");
    }
    DEBUG_PRINTLN(" connected");
    DEBUG_PRINT("IP address: ");
    DEBUG_PRINTLN(WiFi.localIP());
}

void setup() {
    Serial.begin(115200);
    DEBUG_PRINTLN("Setup starting...");

    prefs.begin("zones", false);
    loadState();

    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);
    
    shiftState = 0x0000;
    writeShiftRegister();

    pinMode(OE_PIN, OUTPUT);
    digitalWrite(OE_PIN, LOW); // enable outputs

    shiftState = 0x0000;
    writeShiftRegister();


    connectWifi();

    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    reconnectMqtt();
    applyZones();
}

void loop() {
    if (!mqttClient.connected()) {
        reconnectMqtt();
    }
    mqttClient.loop();

    if (stateChanged && millis() - lastChangeTime >= SAVE_INTERVAL_MS) {
        saveState();
        stateChanged = false;
    }
}

