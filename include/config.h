#ifndef CONFIG_H
#define CONFIG_H

#include "config-private.h"

#define NUM_ZONES 13
#define MASTER_RELAY_INDEX 15

#define COIL_ON_FOR_OPEN 1 // 1 if relay coil should be energized when zone is open
#define ZONE_PULSE_MS 3000

// Set to 0 to disable physical relay activation for testing
#define ACTUATE_RELAYS 1

#endif // CONFIG_H
