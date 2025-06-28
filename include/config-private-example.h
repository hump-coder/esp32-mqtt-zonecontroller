#ifndef CONFIG_PRIVATE_H
#define CONFIG_PRIVATE_H

//
// EXAMPLE config-private.h - PLEASE COPY OR RENAME THIS FILE TO config-private.h AND ADD YOUR OWN CONTENT.
//


#define WIFI_SSID "your wifi ssid"
#define WIFI_PASSWORD "your wifi password"

#define MQTT_SERVER "192.168.1.10" // your mqtt broker
#define MQTT_PORT 1883 // mqtt broker port (default is 1883)
#define MQTT_USER "mqqt-user" // your mqtt user 
#define MQTT_PASSWORD "mqtt-password" // your mqtt user's password

#define DEVICE_NAME "my-zone-controller"
#define BASE_TOPIC "my-zone-controller"

//
// Set to your zone names - these will become part of the mqtt switch entity name in HA..
//
static const char* const ZONE_NAMES[15] = {
    "Lounge", // 1
    "Kitchen", // 2
    "Master Bedroom", //3
    "Zone 4", // 4
    "Zone 5", // 5
    "Zone 6", // 6
    "Zone 7", // 7
    "Zone 8", // 8
    "Zone 9", // 9
    "Zone 10", // 10
    "Zone 11", // 11
    "Zone 12", // 12
    "Zone 13", // 13
    "Zone 14", // 14
    "Zone 15" // 15
};

#endif