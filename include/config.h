#ifndef CONFIG_H
#define CONFIG_H

#include "config-private.h"

#define MASTER_RELAY_INDEX 15
#define MAX_ZONES 15

//
// Set to 0 to disable physical relay activation for testing
//
#define ACTUATE_RELAYS 1

#define MASTER_DELAY 100

//#define HA_PREFIX "homeassistant"
#define HA_PREFIX ""  // Uncomment to disable the prefix

#endif // CONFIG_H
