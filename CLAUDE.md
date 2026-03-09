# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

A Matter-enabled NTC thermistor temperature sensor for ESP32 (configured for ESP32-H2). Reads analog temperature via ADC and exposes it through the Matter protocol over Thread/WiFi.

## Build Commands

This project uses ESP-IDF with CMake. The IDF is located at `/home/tomasmcguinness/esp/esp-idf`.

```bash
idf.py build              # Build the project
idf.py flash              # Flash to device
idf.py monitor            # Monitor serial output
idf.py flash monitor      # Flash and immediately monitor
idf.py menuconfig         # Configure SDK options
idf.py fullclean          # Clean all build artifacts
idf.py set-target esp32h2 # Set target chip (currently esp32h2)
```

The target is ESP32-H2, configured via VS Code settings and `sdkconfig.defaults.esp32h2`.

## Architecture

All application code lives in `main/app_main.cpp`. There is no other custom source code — the project is essentially a thin layer over the `espressif/esp_matter` component.

**Execution flow:**
1. `app_main()` initializes NVS flash and ADC1 (Channel 0, 12dB attenuation)
2. Creates a Matter node with a temperature sensor endpoint (Endpoint 1, `TemperatureMeasurement` cluster)
3. Starts the Matter stack, then spawns a FreeRTOS task for temperature reading

**Temperature reading task (`read_temperature`):**
- Runs every 5 seconds in a loop
- Reads raw ADC value → calibrates to millivolts
- Applies Steinhart-Hart equation: `T = 1 / (1/B * ln(R/R_nominal) + 1/T_nominal) - 273.15`
- Schedules a Matter attribute update on the Matter chip task thread (required — not thread-safe to update directly)

**Thermistor constants** (in `app_main.cpp`):
- `THERMISTORNOMINAL`: 10000 (10kΩ at 25°C)
- `BCOEFFICIENT`: 3969
- `SERIESRESISTOR`: 10000 (10kΩ)
- `ADC1_CHANNEL`: ADC_CHANNEL_0

## Key Files

- `main/app_main.cpp` — all application logic
- `main/idf_component.yml` — declares dependencies (`espressif/esp_matter ^1.4.0`, `esp_bsp_generic ^3`)
- `sdkconfig.defaults` — base SDK defaults (4MB flash, BLE/NimBLE, Matter max 4 endpoints)
- `sdkconfig.defaults.esp32h2` — H2-specific overrides
- `partitions.csv` — flash layout with dual OTA slots and factory NVS

## Matter Commissioning

BLE-WiFi pairing via chip-tool:
```bash
chip-tool pairing ble-wifi 1 <SSID> <PASSPHRASE> 20202021 3840
```

The default discriminator is `3840` and passcode is `20202021`.
