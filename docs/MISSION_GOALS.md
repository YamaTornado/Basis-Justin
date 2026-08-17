# Mission Goals

Der Flight Computer folgt einem festen Datenfluss: Sensoren -> Zustandsschätzung ->
Entscheidung -> {Loggen, Senden}. Jedes Ziel unten entspricht einem Block in diesem
Fluss (siehe `ARCHITECTURE.md`).

1. **Sense** — IMU (LSM9DS1), Barometer (BMP280) und GPS (M100 MINI) zuverlässig mit
   fester Rate auslesen, auch bei einzelnen Sensor-Timeouts.
2. **Estimate** — Rohdaten zu einem konsistenten Zustand fusionieren: Orientierung
   (Attitude) sowie vertikale Höhe/Geschwindigkeit relativ zur Startrampe.
3. **Decide & Act** — Flugphase erkennen (Pad, Boost, Coast, Apogäum, Sinkflug,
   Landung) und Fallschirm-Kanäle (Drogue/Main) sicher und redundant auslösen.
4. **Log** — Rohdaten und geschätzten Zustand kontinuierlich auf Flash schreiben,
   nach dem Flug vollständig auslesbar.
5. **Communicate** — Telemetrie per LoRa senden; der Link darf ausfallen, ohne dass
   Logging oder Deployment davon abhängen.
6. **Verify** — dieselbe Kernlogik (Filter, State Machine, Formate) läuft unverändert
   am Boden: per Replay echter Logfiles und per Simulation synthetischer Flugdaten,
   ganz ohne Hardware.

Nicht-Ziele (bewusst außen vor, um den ersten Wurf klein zu halten): mehrstufige
Raketen, Multi-Board-Redundanz, verschlüsselte Telemetrie, OTA-Updates.
