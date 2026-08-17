# Basis-Justin — Flight Computer

Flight computer firmware for a hobby rocket, built around an ESP32-S3
(4 MB flash / 2 MB PSRAM), ESP-IDF, and FreeRTOS.

See **[docs/MISSION_GOALS.md](docs/MISSION_GOALS.md)** for what this is trying to
do, and **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** for how it's built.

## Layout

```
core/       platform-independent C library: filters, flight state machine,
            log/telemetry formats. No ESP-IDF dependency -- builds and tests
            on the host.
firmware/   ESP-IDF project: sensor/LoRa drivers + FreeRTOS tasks that wire
            core/ to real hardware (Adafruit LSM9DS1, BMP280, HGLRC M100
            MINI GPS, LR-02 LoRa module).
sim/        host-side tool that links core/ directly: run a synthetic
            flight profile, or replay a real flight_log dump -- same code
            path as the firmware, no hardware required.
```

## Building

**Core library + tests (host):**
```sh
cd core
cmake -S . -B build && cmake --build build
ctest --test-dir build --output-on-failure
```

**Simulator:**
```sh
cd sim
cmake -S . -B build && cmake --build build
./build/fc_sim sim                 # synthetic flight
./build/fc_sim replay <logfile>    # replay a real flight_log dump
```

**Firmware (requires the ESP-IDF toolchain, `idf.py`, target `esp32s3`):**
```sh
cd firmware
idf.py set-target esp32s3
idf.py build
idf.py -p <port> flash monitor
```

## Status

Fresh rewrite (previous firmware attempt discarded). Core filters/state
machine/formats are implemented and unit-tested; firmware drivers and task
wiring compile-clean by inspection but are **not yet flashed to real
hardware** -- pin assignments (`firmware/main/pin_config.h`) and the LR-02
AT command set (`firmware/components/drivers/lora_lr02/`) need verification
against the actual boards before a real flight.
