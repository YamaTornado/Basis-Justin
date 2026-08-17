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

    bool read(fc_baro_sample_t *out);

   private:
    Adafruit_BMP280 bmp_;
};
