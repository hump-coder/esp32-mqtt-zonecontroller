#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h>

#include "config.h"
#include "debug.h"

// Pins for SN74HC595 shift registers
const uint8_t DATA_PIN = 14;
const uint8_t CLOCK_PIN = 13;
const uint8_t LATCH_PIN = 12;
const uint8_t OE_PIN = 5; // active low
// Analog pin for ACS712 current sensor
const uint8_t CURRENT_PIN = 35;

// ACS712 characteristics (adjusted for 2k/3.3k level shifter)
// The resistor divider scales the sensor output by R2/(R1+R2) which is
// 3.3k / (2k + 3.3k) ≈ 0.62.  With the sensor centered at 2.5V and
// 100mV/A sensitivity, the ESP32 sees roughly 1.56V offset and 62mV/A.
const float CURRENT_SENSOR_OFFSET = 1.56; // Volts when no current flows
const float CURRENT_SENSOR_SENSITIVITY = 0.062; // Volts per Ampere

// Measured offset in volts after calibration
float currentSensorOffset = CURRENT_SENSOR_OFFSET;

// internal state of shift register (active high relays)
static uint16_t shiftState = 0x0000; // all off (LOW)

// desired zone states (true=open, false=closed)
static bool zoneState[MAX_ZONES] = {false};

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

#define DEVICE_NAME "zone-controller"

const char THING_NAME[] = DEVICE_NAME;
const char INITIAL_AP_PASSWORD[] = "zonezone";
const char CONFIG_VERSION[] = "e1";

DNSServer dnsServer;
WebServer server(80);
IotWebConf iotWebConf(THING_NAME, &dnsServer, &server, INITIAL_AP_PASSWORD, CONFIG_VERSION);

char mqttServer[32] = "192.168.50.11";
char mqttPort[6] = "1883";
char mqttUser[32] = "rinnai";
char mqttPass[32] = "rinnai";
char baseTopic[IOTWEBCONF_WORD_LEN] = DEVICE_NAME;
char numZonesStr[4] = "0";
char pulseSecondsStr[6] = "30";
char defaultZoneStr[4] = "0";
char invertRelaysValue[IOTWEBCONF_WORD_LEN] = "selected";

IotWebConfTextParameter mqttServerParam("MQTT Server", "mqttServer", mqttServer, sizeof(mqttServer), mqttServer, mqttServer);
IotWebConfNumberParameter mqttPortParam("MQTT Port", "mqttPort", mqttPort, sizeof(mqttPort), "1883", "1..65535", "min='1' max='65535' step='1'");
IotWebConfTextParameter mqttUserParam("MQTT User", "mqttUser", mqttUser, sizeof(mqttUser), mqttUser, mqttUser);
IotWebConfPasswordParameter mqttPassParam("MQTT Password", "mqttPassword", mqttPass, sizeof(mqttPass), mqttPass, mqttPass);
IotWebConfTextParameter baseTopicParam("Base Topic", "baseTopic", baseTopic, sizeof(baseTopic), baseTopic, baseTopic);
IotWebConfNumberParameter numZonesParam("Enabled Zones", "numZones", numZonesStr, sizeof(numZonesStr), "0", "0..15", "min='0' max='15'");
IotWebConfNumberParameter pulseSecondsParam("Master Pulse (s)", "pulseSecs", pulseSecondsStr, sizeof(pulseSecondsStr), "30", "1..3600", "min='1' max='3600'");
IotWebConfNumberParameter defaultZoneParam("Default Zone", "defaultZone", defaultZoneStr, sizeof(defaultZoneStr), "0", "0..15", "min='0' max='15'");
IotWebConfCheckboxParameter invertRelaysParam("Invert relay states", "invertRelays", invertRelaysValue, sizeof(invertRelaysValue), true);

uint8_t numZones = 0;
unsigned long zonePulseMs = 30000;
bool coilOnForOpenFlag = true;

Preferences prefs;

const unsigned long SAVE_INTERVAL_MS = 30000;
unsigned long lastChangeTime = 0;
bool stateChanged = false;

// Track current pulse status for master and zone relays
bool pulseActive = false;
unsigned long pulseStartTime = 0;

// Current sensing
float currentAmps = 0.0;
float filteredCurrentAmps = 0.0;
const float CURRENT_FILTER_ALPHA = 0.2f;
unsigned long lastCurrentRead = 0;
const unsigned long CURRENT_READ_INTERVAL_MS = 1000;

void updateConfigVariables();
void handleRoot();
void wifiConnected();
void configSaved();
float readCurrent();
void publishCurrent();

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

bool coilOnForOpen() { return coilOnForOpenFlag; }

bool coilStateForZone(bool open) {
    return open ? coilOnForOpen() : !coilOnForOpen();
}

uint16_t zoneStateToBits() {
    uint16_t val = 0;
    for (uint8_t i = 0; i < numZones; ++i) {
        if (zoneState[i])
            val |= (1 << i);
    }
    return val;
}

void bitsToZoneState(uint16_t bits) {
    for (uint8_t i = 0; i < numZones; ++i) {
        zoneState[i] = bits & (1 << i);
    }
}


void printZoneState() {
    DEBUG_PRINT("Zones state: ");
    for (uint8_t i = 0; i < numZones; ++i) {
        DEBUG_PRINT(zoneState[i] ? "1" : "0");
    }
    DEBUG_PRINTLN("");
}

void updateConfigVariables() {
    numZones = atoi(numZonesStr);
    numZones = 6; // hard coded for test
    if (numZones > MAX_ZONES) numZones = MAX_ZONES;
    zonePulseMs = (unsigned long)atoi(pulseSecondsStr) * 1000;
    coilOnForOpenFlag = !invertRelaysParam.isChecked();
    DEBUG_PRINTLN(String("CONFIG INVERTED: ") + (coilOnForOpenFlag ? "true" : "false"));
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
    snprintf(topic, sizeof(topic), "%s/zone%u/state", baseTopic, zone + 1);
    mqttClient.publish(topic, zoneState[zone] ? "ON" : "OFF", true);

//    DEBUG_PRINT("sending zoneState:");
//    DEBUG_PRINT(zone + 1);
//    DEBUG_PRINT("=");
//    DEBUG_PRINTLN(zoneState[zone] ? "ON" : "OFF");

}

void publishZoneName(uint8_t zone) {
    char topic[64];
    snprintf(topic, sizeof(topic), "%s/zone%u/name", baseTopic, zone + 1);
    String n("ffffuuuu");
    mqttClient.publish(topic, ZONE_NAMES[zone], true);
    //mqttClient.publish(topic, n.c_str(), true);
}

void publishAllStates() {
    for (uint8_t i = 0; i < numZones; ++i) {
        publishZoneState(i);
    }
}

void publishAllZoneNames() {
    for (uint8_t i = 0; i < numZones; ++i) {
        publishZoneName(i);
    }
}


void applyZones() {
    DEBUG_PRINT("Applying zones: ");
    for (uint8_t i = 0; i < numZones; ++i) {
        DEBUG_PRINT(zoneState[i] ? "1" : "0");
#if ACTUATE_RELAYS
        setRelay(i, coilStateForZone(zoneState[i]));
#endif
    }
    DEBUG_PRINTLN("");
    
    publishAllStates();

#if ACTUATE_RELAYS
    writeShiftRegister();

    delay(MASTER_DELAY);
    // ensure master relay is on and start/extend pulse timer
    setRelay(MASTER_RELAY_INDEX, true);
    writeShiftRegister();
    pulseActive = true;
    pulseStartTime = millis();
#else
    DEBUG_PRINTLN(" (dry run - relays not actuated)");
#endif

   // publishAllStates();
}

void updatePulse() {
#if ACTUATE_RELAYS
    if (pulseActive && millis() - pulseStartTime >= zonePulseMs) {
        
        setRelay(MASTER_RELAY_INDEX, false);
        writeShiftRegister();
        delay(MASTER_DELAY);

        for (uint8_t i = 0; i < numZones; ++i) {
            setRelay(i, false);
        }

        writeShiftRegister();
        pulseActive = false;
    }
#endif
}

float readCurrent() {
    const int samples = 250;
    float sumSq = 0.0f;
    for (int i = 0; i < samples; ++i) {
        int raw = analogRead(CURRENT_PIN);
        float voltage = (raw * 3.3f) / 4095.0f;
        float amps = (voltage - currentSensorOffset) / CURRENT_SENSOR_SENSITIVITY;
        sumSq += amps * amps;
        delay(2);
    }
    float rms = sqrt(sumSq / samples);
    // Simple exponential moving average to smooth noise
    filteredCurrentAmps = CURRENT_FILTER_ALPHA * rms +
                         (1.0f - CURRENT_FILTER_ALPHA) * filteredCurrentAmps;
    return filteredCurrentAmps;
}

void publishCurrent() {
    char topic[64];
    snprintf(topic, sizeof(topic), "%s/current", baseTopic);
    char payload[16];
    dtostrf(currentAmps, 0, 2, payload);
    mqttClient.publish(topic, payload, true);
}

void sendDiscovery() {
    DEBUG_PRINT("Sending discovery messages...");
    for (uint8_t i = 0; i < numZones; ++i) {
        char topic[128];
        snprintf(topic, sizeof(topic),
                 "homeassistant/switch/%s/zone%u/config", iotWebConf.getThingName(), i + 1);

        char payload[256];
        snprintf(payload, sizeof(payload),
                "{\"name\":\"%s\",\"command_topic\":\"%s/zone%u/set\",\"state_topic\":\"%s/zone%u/state\",\"uniq_id\":\"%s_zone%u\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\"}",
                ZONE_NAMES[i], baseTopic, i + 1, baseTopic, i + 1,
                iotWebConf.getThingName(), i + 1);
        DEBUG_PRINT("sending payload: ");
        DEBUG_PRINTLN(payload);
        mqttClient.publish(topic, payload, true);
    }

    // Discovery for current sensor
    char curTopic[128];
    snprintf(curTopic, sizeof(curTopic),
             "homeassistant/sensor/%s/current/config", iotWebConf.getThingName());
    char curPayload[256];
    snprintf(curPayload, sizeof(curPayload),
             "{\"name\":\"Current\",\"state_topic\":\"%s/current\",\"unit_of_measurement\":\"A\",\"uniq_id\":\"%s_current\"}",
             baseTopic, iotWebConf.getThingName());
    DEBUG_PRINT("sending payload: ");
    DEBUG_PRINTLN(curPayload);             
    mqttClient.publish(curTopic, curPayload, true);
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
    String prefix = String(baseTopic) + "/zone";
    if (t.startsWith(prefix) && t.endsWith("/set")) {
        int zone = t.substring(prefix.length(), t.length() - 4).toInt();
        if (zone >= 1 && zone <= numZones) {
            bool newState = msg.equalsIgnoreCase("ON") || msg.equalsIgnoreCase("OPEN");
            zoneState[zone - 1] = newState;
            stateChanged = true;
            lastChangeTime = millis();
            applyZones();
        }
    }
}

unsigned long lastMqttAttempt = 0;
bool connectMqtt() {
    if (mqttClient.connected()) return true;

    if (iotWebConf.getState() != iotwebconf::OnLine) {
        return false;
    }

    unsigned long now = millis();
    if (now - lastMqttAttempt < 1000) {
        return false; // limit reconnection attempts
    }
    lastMqttAttempt = now;

    DEBUG_PRINT("Attempting MQTT connection...");
    if (mqttClient.connect(iotWebConf.getThingName(), mqttUser, mqttPass)) {
        DEBUG_PRINTLN("connected");
        String sub = String(baseTopic) + "/+/set";
        mqttClient.subscribe(sub.c_str());
        sendDiscovery();
        publishAllStates();
        publishAllZoneNames();
        publishCurrent();
        return true;
    } else {
        DEBUG_PRINT("failed, rc=");
        DEBUG_PRINTLN(mqttClient.state());
        return false;
    }
}

void wifiConnected() {
    DEBUG_PRINT("WiFi connected IP: ");
    DEBUG_PRINTLN(WiFi.localIP());
    ArduinoOTA.setHostname(iotWebConf.getThingName());
    ArduinoOTA.begin();
    mqttClient.setServer(mqttServer, atoi(mqttPort));
    mqttClient.setCallback(mqttCallback);
    connectMqtt();
  //  applyZones();
}

void configSaved() {
    updateConfigVariables();
}

void handleRoot() {
    DEBUG_PRINTLN("HANDLE ROOT - GOT WEB REQUEST");
    if (iotWebConf.handleCaptivePortal()) {
        return;
    }
    String s = "<!DOCTYPE html><html><body>Go to <a href='config'>configure page</a></body></html>";
    server.send(200, "text/html", s);
}


void setup() {
    Serial.begin(115200);
    DEBUG_PRINTLN("Setup starting...");

    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);
    
    shiftState = 0x0000;
    writeShiftRegister();

    pinMode(OE_PIN, OUTPUT);
    digitalWrite(OE_PIN, LOW); // enable outputs

    shiftState = 0x0000;
    writeShiftRegister();

    pinMode(CURRENT_PIN, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(CURRENT_PIN, ADC_11db);

    DEBUG_PRINTLN("Calibrating current sensor.");
    for (int i = 0; i < 20; ++i) {
        delay(100);
        DEBUG_PRINT(".");
    }
    // Measure sensor offset with no load connected
    const int offsetSamples = 1000;
    long offsetTotal = 0;
    for (int i = 0; i < offsetSamples; ++i) {
        offsetTotal += analogRead(CURRENT_PIN);
        delay(5);
        if(i % 100 == 0) {
            DEBUG_PRINT(".");
        }
    }
    DEBUG_PRINTLN("");
    float offsetAvg = offsetTotal / (float)offsetSamples;
    float offsetVoltage = (offsetAvg * 3.3f) / 4095.0f;
    currentSensorOffset = offsetVoltage;
    DEBUG_PRINT("Calibrated to offsetTotal: ");
    DEBUG_PRINTLN(String(offsetTotal));


    iotWebConf.skipApStartup();
    iotWebConf.setConfigPin(23);
    iotWebConf.setWifiConnectionCallback(&wifiConnected);
    iotWebConf.setConfigSavedCallback(&configSaved);
    iotWebConf.setWifiConnectionTimeoutMs(20000);
    iotWebConf.addSystemParameter(&mqttServerParam);
    iotWebConf.addSystemParameter(&mqttPortParam);
    iotWebConf.addSystemParameter(&mqttUserParam);
    iotWebConf.addSystemParameter(&mqttPassParam);

    iotWebConf.addSystemParameter(&baseTopicParam);
    iotWebConf.addSystemParameter(&numZonesParam);
    iotWebConf.addSystemParameter(&pulseSecondsParam);
    iotWebConf.addSystemParameter(&defaultZoneParam);
    iotWebConf.addSystemParameter(&invertRelaysParam);
    iotWebConf.init();
    updateConfigVariables();
    

    prefs.begin("zones", false);
    loadState();

    //
    // Apply zones now don't wait for network or mqtt connections.
    // we just want to restore the previous state as quickly as we can.
    //
    stateChanged = true;
    lastChangeTime = millis();
    applyZones();

    server.on("/", handleRoot);
    server.on("/config", []{ iotWebConf.handleConfig(); });
    server.onNotFound([](){ iotWebConf.handleNotFound(); });
}

void loop() {

    updatePulse();

    ArduinoOTA.handle();

    iotWebConf.doLoop();

    if (iotWebConf.getState() == iotwebconf::OnLine)
    {
        if (!mqttClient.connected())
        {
            connectMqtt();
        }
        else
        {
            mqttClient.loop();
        }
    }

    // Read and publish current periodically
    if (millis() - lastCurrentRead >= CURRENT_READ_INTERVAL_MS) {
        currentAmps = readCurrent();
        if (mqttClient.connected()) {
            publishCurrent();
        }
        lastCurrentRead = millis();
    }



    if (stateChanged && millis() - lastChangeTime >= SAVE_INTERVAL_MS) {
        saveState();
        stateChanged = false;
    }
}

