#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h>
#include <string.h>

#include "config.h"
#include "debug.h"

//
// This gets set from the IotWebConf
//
bool useShiftRegisters = true;

//
// Pins for SN74HC595 shift registers
// These are only used if the board is configured to use shift registers
//
const uint8_t DATA_PIN = 14;
const uint8_t CLOCK_PIN = 13;
const uint8_t LATCH_PIN = 12;
const uint8_t OE_PIN = 5; // active low

//
// If configured to use gpio's
//
const uint8_t GPIO_ZONE_PINS[8] = {32,33,25,26,27,14,12,13};


//
// internal state of shift register (active high relays)
//
static uint16_t shiftState = 0x0000; // all off (LOW)

//
// desired zone states (true=open, false=closed)
//
static bool zoneState[MAX_ZONES] = {false};

//
// indexes of zones that should remain open by default
//
static uint8_t defaultZones[MAX_ZONES] = {0};
static uint8_t defaultZoneCount = 0;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

#define DEVICE_NAME "8zone-controller"
//#define HA_PREFIX "homeassistant"
#define HA_PREFIX ""  // Uncomment to disable the prefix

const char THING_NAME[] = DEVICE_NAME;
const char INITIAL_AP_PASSWORD[] = "zonezone";
const char CONFIG_VERSION[] = "e3";

DNSServer dnsServer;
WebServer server(80);
IotWebConf iotWebConf(THING_NAME, &dnsServer, &server, INITIAL_AP_PASSWORD, CONFIG_VERSION);

char mqttServer[32] = "192.168.50.11";
char mqttPort[6] = "1883";
char mqttUser[32] = "rinnai";
char mqttPass[32] = "rinnai";
char baseTopic[IOTWEBCONF_WORD_LEN] = DEVICE_NAME;
char haBaseTopic[64];
char numZonesStr[4] = "8";
char pulseSecondsStr[6] = "30";
char defaultZoneStr[32] = "";
char invertRelaysValue[IOTWEBCONF_WORD_LEN] = "selected";
char relayModeValue[IOTWEBCONF_WORD_LEN] = "shift";

static char relayModeValues[][IOTWEBCONF_WORD_LEN] = { "shift", "gpio" };
static char relayModeNames[][15] = { "Shift Register", "GPIO" };

IotWebConfTextParameter mqttServerParam("MQTT Server", "mqttServer", mqttServer, sizeof(mqttServer), mqttServer, mqttServer);
IotWebConfNumberParameter mqttPortParam("MQTT Port", "mqttPort", mqttPort, sizeof(mqttPort), "1883", "1..65535", "min='1' max='65535' step='1'");
IotWebConfTextParameter mqttUserParam("MQTT User", "mqttUser", mqttUser, sizeof(mqttUser), mqttUser, mqttUser);
IotWebConfPasswordParameter mqttPassParam("MQTT Password", "mqttPassword", mqttPass, sizeof(mqttPass), mqttPass, mqttPass);
IotWebConfTextParameter baseTopicParam("Base Topic", "baseTopic", baseTopic, sizeof(baseTopic), baseTopic, baseTopic);
IotWebConfNumberParameter numZonesParam("Enabled Zones", "numZones", numZonesStr, sizeof(numZonesStr), "0", "0..15", "min='0' max='15'");
IotWebConfNumberParameter pulseSecondsParam("Master Pulse (s)", "pulseSecs", pulseSecondsStr, sizeof(pulseSecondsStr), "30", "1..3600", "min='1' max='3600'");
IotWebConfTextParameter defaultZoneParam("Default Zone(s)", "defaultZone", defaultZoneStr, sizeof(defaultZoneStr), "", "Comma separated zone numbers e.g. 1,3,5");
IotWebConfCheckboxParameter invertRelaysParam("Invert relay states", "invertRelays", invertRelaysValue, sizeof(invertRelaysValue), true);
IotWebConfSelectParameter relayModeParam("Relay Mode", "relayMode", relayModeValue, sizeof(relayModeValue), (char*)relayModeValues, (char*)relayModeNames, sizeof(relayModeValues) / IOTWEBCONF_WORD_LEN, sizeof(relayModeNames[0]), "shift");

uint8_t numZones = 0;
unsigned long zonePulseMs = 30000;
bool coilOnForOpenFlag = true;

Preferences prefs;

const unsigned long SAVE_INTERVAL_MS = 30000;
unsigned long lastChangeTime = 0;
bool stateChanged = false;

//
// Track current pulse status for master and zone relays
//
bool pulseActive = false;
unsigned long pulseStartTime = 0;


void updateConfigVariables();
void handleRoot();
void wifiConnected();
void configSaved();

void parseDefaultZones();

void ensureDefaultZonesOpen();

void writeShiftRegister() {
    if (useShiftRegisters) {
        DEBUG_PRINTLN("Were using SHIFT REGISTERS");
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, (shiftState >> 8) & 0xFF);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, shiftState & 0xFF);
        digitalWrite(LATCH_PIN, HIGH);                

    } else {
        DEBUG_PRINTLN("SETTING DESIRED GPIO STATES");
        for (uint8_t i = 0; i < numZones && i < 8; ++i) {
            bool active = shiftState & (1 << i);
            digitalWrite(GPIO_ZONE_PINS[i], active ? HIGH : LOW);
            DEBUG_PRINTLN(String("GPIO: ") + String(GPIO_ZONE_PINS[i]) + (active ? " ON" : " OFF") );
        }
        bool active = shiftState & (1 << MASTER_RELAY_INDEX);
        digitalWrite(GPIO_ZONE_PINS[MASTER_RELAY_INDEX], active ? HIGH : LOW);
        DEBUG_PRINTLN(String("master GPIO: ") + String(GPIO_ZONE_PINS[MASTER_RELAY_INDEX]) + (active ? " ON" : " OFF") );
        DEBUG_PRINTLN("COMPLETED SETTING GPIO STATES.");
    }
}

void debugPrintZones()
{
    DEBUG_PRINT("Zones: ");
    for (uint8_t i = 0; i < numZones; ++i) {
        DEBUG_PRINT(zoneState[i] ? "1" : "0");
    }
    DEBUG_PRINTLN("");
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

void parseDefaultZones() {
    defaultZoneCount = 0;
    const char *p = defaultZoneStr;
    while (*p && defaultZoneCount < MAX_ZONES) {
        while (*p == ' ' || *p == ',') p++;
        if (!*p) break;
        int zone = strtol(p, (char**)&p, 10);
        if (zone >= 1 && zone <= MAX_ZONES) {
            defaultZones[defaultZoneCount++] = zone - 1;
        }
        while (*p == ' ' || *p == ',') p++;
    }
    DEBUG_PRINT("Default zones parsed count: ");
    DEBUG_PRINTLN(defaultZoneCount);
}

void ensureDefaultZonesOpen() {
    uint8_t openCount = 0;
    for (uint8_t i = 0; i < numZones; ++i) {
        if (zoneState[i]) openCount++;
    }
    if (openCount >= defaultZoneCount) return;
    for (uint8_t i = 0; i < defaultZoneCount && openCount < defaultZoneCount; ++i) {
        uint8_t idx = defaultZones[i];
        if (idx < numZones && !zoneState[idx]) {
            zoneState[idx] = true;
            openCount++;
        }
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
    if (numZones > MAX_ZONES) numZones = MAX_ZONES;
    zonePulseMs = (unsigned long)atoi(pulseSecondsStr) * 1000;
    coilOnForOpenFlag = !invertRelaysParam.isChecked();
    useShiftRegisters = strncmp(relayModeValue, "gpio", 4) != 0;

    DEBUG_PRINTLN(String("USE SHIFT REGISTERS CONFIG: ") + String(relayModeValue));

    if (strlen(HA_PREFIX) == 0) {
        strncpy(haBaseTopic, baseTopic, sizeof(haBaseTopic));
        haBaseTopic[sizeof(haBaseTopic) - 1] = '\0';
    } else {
        snprintf(haBaseTopic, sizeof(haBaseTopic), "%s/%s", HA_PREFIX, baseTopic);
    }
    parseDefaultZones();
    DEBUG_PRINTLN(String("CONFIG INVERTED: ") + (coilOnForOpenFlag ? "true" : "false"));
    DEBUG_PRINTLN(String("CONFIG RELAY MODE: ") + (useShiftRegisters ? "shift" : "gpio"));
}


void loadState() {
    uint16_t bits = prefs.getUShort("state", 0);
    bitsToZoneState(bits);
    DEBUG_PRINT("Loaded ");
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
    snprintf(topic, sizeof(topic), "%s/zone%u/state", haBaseTopic, zone + 1);
    mqttClient.publish(topic, zoneState[zone] ? "ON" : "OFF", true);
}

void publishZoneName(uint8_t zone) {
    char topic[64];
    snprintf(topic, sizeof(topic), "%s/zone%u/name", haBaseTopic, zone + 1);
    String n("ffffuuuu");
    mqttClient.publish(topic, ZONE_NAMES[zone], true);
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
    ensureDefaultZonesOpen();
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
    DEBUG_PRINTLN("ZONE RELAYS ENERGISED TO REQUIRED STATES, NOW ENERGISING MASTER TO SUPPLY POWER");
    delay(MASTER_DELAY);
    // ensure master relay is on and start/extend pulse timer
    setRelay(MASTER_RELAY_INDEX, true);
    writeShiftRegister();
    pulseActive = true;
    DEBUG_PRINTLN("MASTER RELAY ENERGISED - WILL HOLD FOR MASTER PULSE TIME.");    
    pulseStartTime = millis();
#else
    DEBUG_PRINTLN(" (dry run - relays not actuated)");
#endif
   
}

void updatePulse() {
#if ACTUATE_RELAYS
    if (pulseActive && millis() - pulseStartTime >= zonePulseMs) {
        
        DEBUG_PRINTLN("MASTER RELAY PULSE TIME EXPIRED, LET'S DE-ENERGISED.");

        setRelay(MASTER_RELAY_INDEX, false);
        writeShiftRegister();
        DEBUG_PRINTLN("MASTER RELAY DE-ENERGISED");
        delay(MASTER_DELAY);

        for (uint8_t i = 0; i < numZones; ++i) {
            setRelay(i, false);
        }

        writeShiftRegister();
        pulseActive = false;
        DEBUG_PRINTLN("ZONE RELAY DE-ENERGISE COMPLETE");
    }
#endif
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
                ZONE_NAMES[i], haBaseTopic, i + 1, haBaseTopic, i + 1,
                iotWebConf.getThingName(), i + 1);
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
    String prefix = String(haBaseTopic) + "/zone";
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
        String sub = String(haBaseTopic) + "/+/set";
        mqttClient.subscribe(sub.c_str());
        sendDiscovery();
        publishAllStates();
        publishAllZoneNames();
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
    iotWebConf.addSystemParameter(&relayModeParam);
    iotWebConf.init();
    updateConfigVariables();

    if (useShiftRegisters) {
        DEBUG_PRINTLN("SETTING UP BOARD TO USE SHIFT REGISTERS FOR RELAYS.");

        pinMode(DATA_PIN, OUTPUT);
        pinMode(CLOCK_PIN, OUTPUT);
        pinMode(LATCH_PIN, OUTPUT);
        pinMode(OE_PIN, OUTPUT);
        digitalWrite(OE_PIN, LOW); // enable outputs
    } else {
        DEBUG_PRINTLN("SETTING UP BOARD TO USE GPIO FOR RELAYS.");

        //
        // Let's setup all 8 relays ready for use (even if they're not enabled)
        //
        for (uint8_t i = 0; i < 8; ++i) {
            pinMode(GPIO_ZONE_PINS[i], OUTPUT);
            digitalWrite(GPIO_ZONE_PINS[i], LOW);
        }
    }

  //  DEBUG_PRINTLN("CLEARING ALL RELAYS (DEFAULT START STATE BEFORE STATE LOADED.");
    shiftState = 0x0000;
    //writeShiftRegister();
    
    prefs.begin("zones", false);
    loadState();
    DEBUG_PRINTLN("PERSISTED STATE LOADED.");

    //
    // Apply zones now don't wait for network or mqtt connections.
    // we just want to restore the previous state as quickly as we can.
    //
    stateChanged = true;
    lastChangeTime = millis();
    DEBUG_PRINTLN("APPLYING LOADED STATE.");
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

    if (stateChanged && millis() - lastChangeTime >= SAVE_INTERVAL_MS) {
        saveState();
        stateChanged = false;
    }
}

