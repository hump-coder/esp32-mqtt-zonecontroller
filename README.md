# ESP32 MQTT Zone Controller

This project controls multiple zone relays using an ESP32 and MQTT. It is designed for systems like irrigation or HVAC where a master valve or pump is triggered along with up to 15 individual zones. Zone states are persisted between reboots and the device can be configured over Wi‑Fi.

## Features

- Supports up to **15** zones plus a master relay driven through SN74HC595 shift registers.
- MQTT control topics for each zone with retained state publication.
- Automatic Home Assistant discovery messages.
- Web based configuration of Wi‑Fi and MQTT settings via IotWebConf.
- Adjustable master relay pulse time and option to invert relay logic.
- State stored in NVS so zones resume their last state after power loss.
- Dry‑run mode by setting `ACTUATE_RELAYS` to `0` in `include/config.h`.
- Self‑calibrates its current sensor at boot, so start the controller with no load connected.
- Zone display names defined in `include/config.h` and published over MQTT.

## Hardware

Shift register wiring uses the following pins:

| Function | GPIO |
|---------|-----|
| DATA    | 14  |
| CLOCK   | 13  |
| LATCH   | 12  |
| OE (active low) | 5 |

The configuration button is on GPIO23. `MASTER_RELAY_INDEX` is `15`.

## MQTT Topics

The default base topic is `zone-controller`. Each zone `n` (1‑15) listens for commands on:

```
<baseTopic>/zone<n>/set    (payload `ON` or `OFF`)
```

Current state is published to:

```
<baseTopic>/zone<n>/state  (payload `ON` or `OFF`)
```

Each zone's configured name is also published when MQTT connects:

```
<baseTopic>/zone<n>/name   (payload is the display name)
```

Home Assistant discovery is sent under `homeassistant/switch/<device>/zone<n>/config` when MQTT connects.

## Building

Install [PlatformIO](https://platformio.org) and run:

```
pio run            # build
pio run -t upload  # flash the ESP32
pio device monitor # optional serial monitor
```

The project targets the `esp32dev` board using the Arduino framework. See `platformio.ini` for details.

### OTA Updates

After the device is connected to Wi‑Fi, it exposes an Arduino OTA service using
its configured device name. You can upload firmware over the network using
PlatformIO:

```
pio run -t upload --upload-port <device-hostname>.local
```

Replace `<device-hostname>` with the value shown in the configuration interface.

## Configuration

On first boot (or when the config button is held) the controller starts an access point named `zone-controller` with password `zonezone`. Browse to `/config` to enter Wi‑Fi credentials, MQTT broker information, number of zones, master pulse length and other options. Settings are saved in flash and restored on reboot.

## License

This project is released under the MIT License. See [LICENSE](LICENSE) for details.
