#ifndef CONFIG_H
#define CONFIG_H

#define MASTER_RELAY_INDEX 15
#define MAX_ZONES 15

//#define ZONE_COUNT 6

// Set to 0 to disable physical relay activation for testing
#define ACTUATE_RELAYS 1

#define MASTER_DELAY 100

static const char* const ZONE_NAMES[MAX_ZONES] = {
    "xKids", // 1
    "xMaster", // 2
    "xKarl's Office", //3
    "xOffice", // 4
    "xKitchen", // 5
    "xBedroom", // 6
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

#endif // CONFIG_H
