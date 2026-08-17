# Basis-Justin — Flight Computer

Flight computer firmware for a hobby rocket, built around an ESP32-S3
(4 MB flash / 2 MB PSRAM). OS is FreeRTOS/ESP-IDF; the Arduino core runs on
top of it (via PlatformIO) purely for library access (Adafruit sensor
libraries, TinyGPS++) -- FreeRTOS tasks are still created explicitly, not
`setup()`/`loop()` polling.

See **[docs/MISSION_GOALS.md](docs/MISSION_GOALS.md)** for what this is trying to
do, and **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** for how it's built.

## Layout

```
core/       platform-independent C library: filters, flight state machine,
            log/telemetry formats. No ESP-IDF/Arduino dependency -- builds
            and tests on the host.
firmware/   PlatformIO project (framework = arduino, on ESP-IDF/FreeRTOS
            underneath): sensor/LoRa driver wrappers + FreeRTOS tasks that
            wire core/ to real hardware (Adafruit LSM9DS1, BMP280, HGLRC
            M100 MINI GPS via TinyGPS++, LR-02 LoRa module).
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

**Firmware (requires [PlatformIO](https://platformio.org/) -- e.g. the
pioarduino IDE/CLI):**
```sh
cd firmware
pio run                # build
pio run -t upload      # flash
pio device monitor      # serial log
```

## Status

Fresh rewrite (previous firmware attempt discarded). Core filters/state
machine/formats are implemented and unit-tested; firmware drivers and task
wiring compile-clean by inspection but are **not yet built with PlatformIO
or flashed to real hardware** (no network access to PlatformIO/Arduino
toolchains in the environment this was written in). Known things to verify
before a real flight:

- Pin assignments: `firmware/include/pin_config.h`.
- LR-02 AT command set: `firmware/lib/lora_lr02/` (biggest unverified
  assumption in the project -- see docs/ARCHITECTURE.md).
- `board_build.*` flash/PSRAM settings in `firmware/platformio.ini` against
  the actual ESP32-S3-FH4R2 (currently based on the closest stock
  `esp32-s3-devkitc-1` board definition with overrides).
