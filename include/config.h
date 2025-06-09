#ifndef CONFIG_H
#define CONFIG_H

//
// REMOVE THE config-private.h INCLUDE AND DEFINE THE SECRETS BELOW
//
#include "config-private.h"

//#define WIFI_SSID "your-ssid"
//#define WIFI_PASSWORD "your-password"

//#define MQTT_SERVER "your-mqtt-broker"
//#define MQTT_PORT 1883
//#define MQTT_USER "username"
//#define MQTT_PASSWORD "password"

//#define DEVICE_NAME "ac-zone-controller"
//#define BASE_TOPIC "ac-zone-controller"


#define NUM_ZONES 13
#define MASTER_RELAY_INDEX 15

#define COIL_ON_FOR_OPEN 1 // 1 if relay coil should be energized when zone is open
#define ZONE_PULSE_MS 3000

// Set to 0 to disable physical relay activation for testing
#define ACTUATE_RELAYS 1

#endif // CONFIG_H
