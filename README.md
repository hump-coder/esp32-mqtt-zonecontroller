# ESP32 MQTT Zone Controller

This project controls multiple zone relays using an ESP32 and MQTT. It is designed for systems like irrigation or HVAC where a master valve or pump is triggered along with up to 15 individual zones. Zone states are persisted between reboots and the device can be configured over Wi‑Fi.

## Features

- Supports up to **15** zones plus a master relay driven through SN74HC595 shift registers. A GPIO mode is also available for simpler 8 channel boards.
- MQTT control topics for each zone with retained state publication.
- Automatic Home Assistant discovery messages.
- Web based configuration of Wi‑Fi and MQTT settings via IotWebConf.
- Adjustable master relay pulse time and option to invert relay logic.
- State stored in NVS so zones resume their last state after power loss.
- Dry‑run mode by setting `ACTUATE_RELAYS` to `0` in `include/config.h`.
- Zone display names defined in `include/config.h` and published over MQTT.

## Hardware

The GPIO configuration is targeting this 8 relay ESP32 board (but any should work fine with the correct GPIO mappings)
![image](https://github.com/user-attachments/assets/2a5ab7c2-74e4-4811-a08b-7ef5923f3de3)

The shift register configuration is targeting this 16 relay ESP32 board:
![image](https://github.com/user-attachments/assets/8cf27e2f-2b09-4340-9b8b-3ca27deb4131)

Example Hacky Crude implementation (yes, this is one of mine!)

![image](https://github.com/user-attachments/assets/c4e541f4-674f-409d-98e6-ddceb3e50d6d)
![image](https://github.com/user-attachments/assets/58b11896-cf07-40e4-b8a4-cac35bb5c93a)
![image](https://github.com/user-attachments/assets/2fcf734a-7838-4372-8aaa-86bb81c576b6)
![image](https://github.com/user-attachments/assets/ba1d5e8e-7ea0-4f31-9fcd-a9db3452e254)
![image](https://github.com/user-attachments/assets/73a0ada8-4132-4bd9-b6ea-69926773fad0)

Shift register wiring uses the following pins:

| Function | GPIO |
|---------|-----|
| DATA    | 14  |
| CLOCK   | 13  |
| LATCH   | 12  |
| OE (active low) | 5 |

The configuration button is on GPIO23. `MASTER_RELAY_INDEX` is `15`.

When `Relay Mode` is set to **GPIO** the first 8 zones are mapped to the
following pins by default:

| Zone | GPIO |
|-----|-----|
| 1 | 32 |
| 2 | 33 |
| 3 | 25 |
| 4 | 26 |
| 5 | 27 |
| 6 | 14 |
| 7 | 12 |
| 8 | 13 |

## MQTT Topics

The default base topic is `8zone-controller`. Topics are automatically published under the `homeassistant/` prefix. This prefix can be removed by setting `HA_PREFIX` to an empty string in `src/main.cpp`. Each zone `n` (1‑15) listens for commands on:

```
homeassistant/<baseTopic>/zone<n>/set    (payload `ON` or `OFF`)
```

Current state is published to:

```
homeassistant/<baseTopic>/zone<n>/state  (payload `ON` or `OFF`)
```

Each zone's configured name is also published when MQTT connects:

```
homeassistant/<baseTopic>/zone<n>/name   (payload is the display name)
```

Home Assistant discovery is sent under `homeassistant/switch/<device>/zone<n>/config` when MQTT connects.

## Building

Copy `include/config-private-example.h` to `include/config-private.h` and populate it with your Wi-Fi, MQTT and zone configuration.

Install [PlatformIO](https://platformio.org) and run:

```
pio run            # build
pio run -t upload  # flash the ESP32
pio device monitor # optional serial monitor
```

You can also run `scripts/setup.sh` for a one-off build which copies the example configuration and uses the `PIO_BOARD` environment variable if set.

The provided `platformio.ini` targets the `esp32dev` board using the Arduino framework. If you have a different ESP32 based board set the `board` option accordingly or export `PIO_BOARD=<your-board>` before running PlatformIO. See `platformio.ini` for valid board IDs.

### OTA Updates

After the device is connected to Wi‑Fi, it exposes an Arduino OTA service using
its configured device name. You can upload firmware over the network using
PlatformIO:

```
pio run -t upload --upload-port <device-hostname>.local
```

Replace `<device-hostname>` with the value shown in the configuration interface.

## Configuration

On first boot (or when the config button is held) the controller starts an access point named `8zone-controller` with password `zonezone`. Browse to `/config` to enter Wi‑Fi credentials, MQTT broker information, number of zones, master pulse length and other options. Settings are saved in flash and restored on reboot.

## License

This project is released under the MIT License. See [LICENSE](LICENSE) for details.
