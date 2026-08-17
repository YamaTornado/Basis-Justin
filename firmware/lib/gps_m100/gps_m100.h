#pragma once

#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

#include "fc/types.h"

/* HGLRC M100 MINI: u-blox-based GPS module, NMEA-0183 over UART (9600 baud
 * by default on most M100 MINI units -- verify against your module, some
 * ship pre-configured for a different rate). Parses via TinyGPS++ instead
 * of a hand-rolled NMEA parser. */
class GpsM100 {
   public:
    /* serial: which HardwareSerial to use (e.g. Serial1), already
     * .begin()'d by the caller is NOT required -- this calls begin() with
     * rx/tx pins itself. */
    bool begin(HardwareSerial &serial, int rx_gpio, int tx_gpio, int baud = 9600);

    /* Feeds any bytes currently waiting on the UART into TinyGPS++ and
     * writes the latest known fix into *out. Returns true if at least one
     * NMEA sentence was successfully parsed during this call (out is
     * always populated with the latest known values regardless). */
    bool poll(fc_gps_sample_t *out);

   private:
    HardwareSerial *serial_ = nullptr;
    TinyGPSPlus gps_;
    fc_gps_sample_t last_{};
};
