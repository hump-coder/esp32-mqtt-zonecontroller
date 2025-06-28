#!/bin/bash

set -e

# Determine board from PIO_BOARD env variable or fallback
BOARD="${PIO_BOARD:-esp32dev}"

if [ ! -f include/config-private.h ]; then
    cp include/config-private-example.h include/config-private.h
    echo "Created include/config-private.h from example."
else
    echo "include/config-private.h already exists."
fi

if ! grep -q "^board =" platformio.ini; then
    echo "board = $BOARD" >> platformio.ini
    echo "Added board setting to platformio.ini"
fi

platformio run

