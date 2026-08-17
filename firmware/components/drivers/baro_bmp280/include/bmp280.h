#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "fc/types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMP280_ADDR_DEFAULT 0x76 /* 0x77 if SDO is pulled high instead of low */

typedef struct {
    i2c_master_dev_handle_t dev;
    /* calibration coefficients, read once at init (see datasheet 3.11.2) */
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} bmp280_t;

esp_err_t bmp280_init(bmp280_t *dev, i2c_master_bus_handle_t bus, uint16_t addr);

/* Blocking read of one pressure+temperature sample (normal mode, device is
 * continuously converting per the oversampling/standby config set in
 * bmp280_init()). */
esp_err_t bmp280_read(bmp280_t *dev, fc_baro_sample_t *out);

/* Converts a pressure reading to altitude above a reference pressure using
 * the international barometric formula. Both pressures in Pa. */
float bmp280_pressure_to_altitude_m(float pressure_pa, float reference_pressure_pa);

#ifdef __cplusplus
}
#endif
