#ifndef CONFIG_H
#define CONFIG_H

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

#define MQTT_SERVER "YOUR_MQTT_SERVER"
#define MQTT_PORT 1883
#define MQTT_USER ""
#define MQTT_PASSWORD ""

#define DEVICE_NAME "ac-zone-controller"
#define BASE_TOPIC "ac-zone-controller"

#define NUM_ZONES 13
#define MASTER_RELAY_INDEX 15

#define COIL_ON_FOR_OPEN 0 // 1 if relay coil should be energized when zone is open
#define ZONE_PULSE_MS 3000

// Set to 0 to disable physical relay activation for testing
#define ACTUATE_RELAYS 1

#endif // CONFIG_H
