#pragma once

#include <Adafruit_BMP280.h>

#include "fc/types.h"

/* Thin wrapper around Adafruit's BMP280 library. Same role as
 * imu_lsm9ds1.h: owns the sensor object, exposes fc_baro_sample_t so the
 * rest of the firmware never touches Adafruit_BMP280 directly. */
class BaroBmp280 {
   public:
    /* addr: 0x76 or 0x77 depending on the breakout's SDO strap -- verify
     * against your board (0x76 is the more common default). */
    bool begin(uint8_t addr = 0x76);

    /* Returns false immediately (without touching the bus) if begin()
     * never succeeded -- hammering a sensor that isn't there with I2C
     * transactions every cycle at 100Hz has been observed to eventually
     * crash the Adafruit_BMP280 library rather than just fail cleanly. */
    bool read(fc_baro_sample_t *out);

   private:
    Adafruit_BMP280 bmp_;
    bool began_ = false;
};
