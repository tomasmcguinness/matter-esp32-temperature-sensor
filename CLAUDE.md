# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Matter-enabled NTC thermistor temperature sensor firmware for ESP32 variants (ESP32, ESP32-C3, ESP32-H2). Built on ESP-IDF with the espressif/esp_matter component (^1.4.0). Licensed MIT.

## Build Commands

Requires ESP-IDF environment to be sourced (`. $IDF_PATH/export.sh` or equivalent).

```bash
idf.py set-target esp32c3      # Set chip target (run before first build)
idf.py build                   # Build firmware
idf.py flash                   # Flash to connected device
idf.py monitor                 # Serial monitor (Ctrl+] to exit)
idf.py flash monitor           # Flash then monitor
idf.py menuconfig              # Interactive Kconfig editor
```

## Architecture

### Stack
- **ESP-IDF** (v5.0+) — build system, drivers, FreeRTOS
- **esp_matter** — Matter protocol (commissioning, clusters, attribute updates)
- **C++17** with ESP-IDF CMake build system

### Application (`main/app_main.cpp`)

Single-file firmware with three logical sections:

1. **ADC Temperature Reading** — NTC thermistor on ADC1 Channel 0 with 12dB attenuation. Uses Steinhart-Hart equation (B=3969, R₀=10kΩ @ 25°C, series resistor 10kΩ). ADC calibration via eFuse or curve fitting.

2. **Matter Device Setup** — Creates root node (endpoint 0) + temperature sensor endpoint (endpoint 1) with `TemperatureMeasurement` cluster. Temperature is written as `int16` (value × 100) to `MeasuredValue` attribute.

3. **FreeRTOS Task (`read_temperature`)** — Reads ADC every 5 seconds, computes temperature, and schedules attribute update on the Matter thread via `SystemLayer().ScheduleLambda()` for thread safety.

### Commissioning Flow
- BLE pairing opens automatically if no fabrics exist (300s window)
- After pairing, switches to DNS-SD advertisement
- Commission via: `chip-tool pairing ble-wifi 1 <SSID> <PASSPHRASE> 20202021 3840`
- Read values: `chip-tool interactive start` → `temperaturemeasurement subscribe measured-value 3 10 1 1`

### Configuration
- `sdkconfig.defaults` — base config (WiFi AP disabled, OTA enabled, NimBLE)
- `sdkconfig.defaults.esp32c3` / `sdkconfig.defaults.esp32h2` — chip-specific overrides
- `partitions.csv` — custom partition table with dual OTA slots (1.875MB each)
- `main/Kconfig.projbuild` — menuconfig options for pin assignments

### Hardware Design
KiCad schematic and PCB in `Hardware/` directory. Not relevant to firmware development.

## Key Dependencies

Managed via `main/idf_component.yml`:
- `espressif/esp_matter: ^1.4.0`
- `esp_bsp_generic: ^3`
- `espressif/cmake_utilities: ^1`

## No Tests

There is no test suite. This is a hardware-focused embedded project.
