#ifndef CONFIG_H
#define CONFIG_H



#define MASTER_RELAY_INDEX 15
#define MAX_ZONES 15

// Set to 0 to disable physical relay activation for testing
#define ACTUATE_RELAYS 1

#define MASTER_DELAY 100

static const char* const ZONE_NAMES[MAX_ZONES] = {
    "Zone 1", "Zone 2", "Zone 3", "Zone 4", "Zone 5",
    "Zone 6", "Zone 7", "Zone 8", "Zone 9", "Zone 10",
    "Zone 11", "Zone 12", "Zone 13", "Zone 14", "Zone 15"
};

#endif // CONFIG_H
