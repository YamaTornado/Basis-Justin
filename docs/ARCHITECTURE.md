# Architecture

## Leitidee

Die eigentliche Flugsoftware (Filter, State Machine, Log-/Telemetrieformate) ist als
**plattformunabhängige C-Library** (`core/`) von den treiber-/task-spezifischen Teilen
(`firmware/`) getrennt. `core/` hat keine ESP-IDF- oder Arduino-Abhängigkeit und lässt
sich mit einem normalen Host-Compiler bauen und testen.

Daraus folgt praktisch kostenlos:

- **Unit-Tests** für Filter/State-Machine am PC, ohne Hardware, ohne Emulator.
- **Simulation**: `sim/` linkt `core/` und füttert es mit synthetischen
  Flugprofilen (z.B. parabolischer Höhenverlauf + Rauschen).
- **Replay**: dasselbe `sim/`-Tool kann statt synthetischer Daten ein echtes,
  auf dem Gerät geschriebenes Logfile einspielen — gleicher Code-Pfad wie im Flug.

`firmware/` läuft auf **FreeRTOS/ESP-IDF als Betriebssystem**, mit dem **Arduino-Core
als Bibliothek obendrauf** (PlatformIO, `framework = arduino`, siehe unten) — nicht
umgekehrt. Seit arduino-esp32 3.x ist der Arduino-Core selbst auf ESP-IDF 5.x gebaut,
d.h. `framework = arduino` liefert bereits FreeRTOS (`xTaskCreate`, `xQueueCreate`, ...)
und bei Bedarf direkten Zugriff auf ESP-IDF-APIs (`esp_partition`, `esp_timer`, ...) —
zusätzlich zum kompletten Arduino-Library-Ökosystem (Adafruit-Sensor-Libraries,
TinyGPS++, ...). Die fünf FreeRTOS-Tasks (`sensor_task`, `estimator_task`,
`flight_task`, `logger_task`, `telemetry_task`) werden weiterhin explizit selbst
angelegt (kein `setup()`/`loop()`-Polling) — Arduino liefert nur die Treiber-Bibliotheken,
nicht die Nebenläufigkeits-Architektur.

```
core/                         firmware/ (PlatformIO, framework=arduino)   sim/
┌───────────────────┐         ┌──────────────────────────┐         ┌───────────────────┐
│ attitude_filter    │  <───── │ lib/tasks/estimator_task  │         │ main.c            │
│ altitude_filter    │         │ lib/tasks/sensor_task      │         │ profiles.c        │
│ flight_state       │  <───── │ lib/tasks/flight_task       │        │ (linkt core/ neu) │
│ log_format         │  <───── │ lib/tasks/logger_task        │       └───────────────────┘
│ telemetry          │  <───── │ lib/tasks/telemetry_task      │
│ types.h            │         │ lib/{imu,baro,gps,lora}_*      │
└───────────────────┘         │   (Adafruit/TinyGPS++ Wrapper)  │
   reines C, keine            └──────────────────────────┘
   ESP-IDF/Arduino-Abh.          FreeRTOS (ESP-IDF) + Arduino-Libraries
```

## Datenfluss (Firmware)

```
 IMU ─┐
 Baro ┼─> sensor_task ──(queue: raw samples)──> estimator_task ──(queue: state)──┬─> flight_task   -> Pyro GPIOs
 GPS ─┘                                          (Madgwick + Altitude-KF)         ├─> logger_task   -> Flash-Partition
                                                                                   └─> telemetry_task -> LoRa (UART/AT)
```

Jeder Pfeil ist eine FreeRTOS-Queue (kein geteilter globaler Zustand). Jeder Task hat
eine feste Rate und blockiert nicht auf einen anderen — ein langsamer LoRa-Versand
darf niemals das Deployment verzögern. Deshalb hat `flight_task` die höchste Priorität,
`telemetry_task` die niedrigste.

| Task            | Rate     | Priorität | Quelle/Ziel                          |
|-----------------|----------|-----------|---------------------------------------|
| sensor_task     | 100 Hz   | hoch      | I2C via `Wire`+Adafruit-Libs (IMU, Baro), UART via TinyGPS++ (GPS, langsamer) |
| estimator_task  | 100 Hz   | hoch      | konsumiert Rohdaten, produziert State  |
| flight_task     | 100 Hz   | **höchste** | State -> Phase + Pyro-GPIOs (`digitalWrite`) |
| logger_task     | 50 Hz    | mittel    | State/Raw -> Flash-Partition (Ring, `esp_partition`) |
| telemetry_task  | 5–10 Hz  | niedrig   | State -> LoRa (AT-Commands via `HardwareSerial`) |

## Build-System: PlatformIO (Arduino-Framework)

`firmware/` ist ein PlatformIO-Projekt (`firmware/platformio.ini`), `framework = arduino`,
Plattform-Paket von [pioarduino](https://github.com/pioarduino/platform-espressif32)
(aktuellere arduino-esp32-3.x-Releases als das offizielle PlatformIO-Registry-Paket).
Layout folgt PlatformIO-Konvention statt ESP-IDF-Komponenten:

```
firmware/
  platformio.ini
  partitions.csv          -- wie zuvor, per board_build.partitions eingebunden
  include/pin_config.h     -- projektweite Pin-Belegung
  src/main.cpp             -- setup(): Wire/Sensoren/Queues/Tasks anlegen; loop() ungenutzt
  lib/
    imu_lsm9ds1/            -- Wrapper um Adafruit_LSM9DS1
    baro_bmp280/             -- Wrapper um Adafruit_BMP280
    gps_m100/                 -- Wrapper um TinyGPS++
    lora_lr02/                 -- AT-Kommandos über HardwareSerial (kein Arduino-Lib nötig)
    flash_log/                  -- esp_partition-Ringpuffer (ESP-IDF-API, via Arduino erreichbar)
    tasks/                       -- die 5 FreeRTOS-Tasks + Queues, wie oben
```

`core/` wird per `lib_deps = symlink://../core` eingebunden (keine Kopie) — PlatformIOs
Konvention `include/` + `src/` passt zufällig exakt zum bestehenden CMake-Layout von
`core/`, d.h. dieselbe Library wird von Host-Tests, `sim/` und der Firmware geteilt.

Jeder Treiber-Wrapper in `lib/` exportiert nur `fc/types.h`-Structs (`fc_imu_sample_t`
usw.) nach außen — der Rest der Firmware (Tasks) sieht nie eine `Adafruit_*`- oder
`TinyGPSPlus`-Klasse direkt. Das hält `core/` weiterhin komplett bibliotheksfrei und
macht einen späteren Wechsel der Sensor-Library lokal auf eine Datei begrenzt.

## Estimation: pragmatisch statt Voll-EKF

Statt eines einzelnen 15-Zustands-EKF (teuer, schwer zu tunen, auf einem ESP32 mit
begrenztem RAM unnötig komplex) zwei einfachere, gut verstandene Filter kombiniert:

1. **Attitude**: Madgwick-Filter (Gyro+Accel+Mag) -> Orientierungs-Quaternion.
   Damit wird die rohe Beschleunigung ins Weltkoordinatensystem rotiert
   (Gravitation entfernt) -> vertikale Beschleunigung.
2. **Altitude/Velocity**: linearer 3-Zustands-Kalman-Filter
   `[Höhe, vertikale Geschwindigkeit, Beschleunigungs-Bias]`, Prädiktion mit der
   vertikalen Beschleunigung aus (1), Korrektur mit Barometerhöhe (und optional,
   stark gewichtet, GPS-Höhe).

Das `fc_state_estimate_t`-Interface bleibt stabil — die Filter lassen sich später
durch ein echtes EKF ersetzen, ohne dass State-Machine, Logger oder Telemetrie sich
ändern müssen.

## Flugphasen (`flight_state`)

```
PAD -> ARMED -> BOOST -> COAST -> APOGEE -> DROGUE_DESCENT -> MAIN_DESCENT -> LANDED
                                                      \-> ABORT (Sicherheits-Fallback)
```

- **BOOST**: vertikale Beschleunigung übersteigt Schwelle für N aufeinanderfolgende
  Samples (Entprellung gegen Vibration/Ausreißer).
- **APOGEE**: vertikale Geschwindigkeit wechselt von + nach -, ebenfalls entprellt.
  Zusätzlich ein **Backup-Timer**: falls kein Vorzeichenwechsel erkannt wird, löst
  Drogue nach `t_boost + max_coast_time_ms` trotzdem aus (klassisches Redundanz-Muster
  in Hobby-Raketen-Flugcomputern).
- **MAIN_DESCENT**: Drogue ist offen und Höhe über Grund unterschreitet
  `main_deploy_altitude_m`.
- GPIO-Zugriff liegt **nicht** in `core/` — `flight_state` ruft einen Funktionszeiger
  (`fc_pyro_fire_fn`) auf, den die Firmware beim Init registriert. So bleibt `core/`
  hardwarefrei und im State-Machine-Unit-Test lässt sich das "Auslösen" einfach
  mitzählen statt echte Pins zu schalten.

## Logging

4 MB Flash sind knapp (App + NVS + Log-Partition müssen sich das teilen). Statt eines
vollen Filesystems (SPIFFS/LittleFS, Overhead) schreibt `logger_task` **Rohbytes in
eine dedizierte Flash-Partition** (siehe `partitions.csv`) als Ringpuffer aus kompakten,
getypten Records (`log_format.h`): Header {type, len, timestamp} + Payload + CRC8.
Nach dem Flug liest ein Host-Tool (später, `tools/log_dump`) die Partition aus und
dekodiert sie mit demselben `log_format.c` wie die Firmware — Skript und Firmware
können nicht auseinanderlaufen.

## Telemetrie

Kompaktes Binärformat (`telemetry.h`): Sync-Bytes, Sequenznummer, Typ, Länge, CRC8,
Payload. Der LoRa-Treiber (`lora_lr02`) kapselt die AT-Befehle für das LR-02-Modul;
die genauen AT-Kommandos sind im Treiber als klar markierte Annahmen dokumentiert und
müssen gegen das Datenblatt verifiziert werden — das ist der Teil mit dem größten
Unsicherheitsfaktor im gesamten Projekt.

## Flash-Partitionierung (4 MB, vorläufig)

| Partition | Größe   | Zweck                    |
|-----------|---------|---------------------------|
| nvs       | 24 KB   | Konfiguration/Kalibrierung |
| phy_init  | 4 KB    | RF-Kalibrierung (ESP-IDF)  |
| factory   | ~1.5 MB | Firmware-Image             |
| flight_log| ~2.4 MB | Ringpuffer für Flugdaten   |

## Warum PSRAM (2 MB)?

Wird primär für größere Puffer gebraucht, die nicht ins interne RAM sollen: mehrere
Sekunden Rohdaten-Ringpuffer für den Logger (Spitzenlast abfedern, falls Flash-Write
kurz blockiert) sowie Puffer für spätere Erweiterungen (z.B. Kameras/größere
Telemetrie-Pakete). Für die Kernlogik (Filter, State Machine) wird bewusst kein PSRAM
vorausgesetzt, damit `core/` auch auf kleineren Targets liefe.
