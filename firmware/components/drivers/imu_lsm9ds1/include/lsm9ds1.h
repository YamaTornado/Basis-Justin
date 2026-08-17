#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "fc/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Adafruit LSM9DS1 breakout: accel+gyro and mag are two separate I2C
 * devices on the same bus. The Adafruit board ties SDO_AG/SDO_M high, so
 * the default addresses (0x6B, 0x1E) apply -- double check against your
 * specific board/wiring if you get NACKs on init. */
#define LSM9DS1_ADDR_AG_DEFAULT 0x6B
#define LSM9DS1_ADDR_M_DEFAULT 0x1E

typedef struct {
    i2c_master_dev_handle_t ag; /* accel + gyro device */
    i2c_master_dev_handle_t m;  /* magnetometer device */
} lsm9ds1_t;

typedef struct {
    uint16_t addr_ag;
    uint16_t addr_m;
} lsm9ds1_config_t;

void lsm9ds1_config_default(lsm9ds1_config_t *cfg);

/* Adds the AG and M devices to bus and configures both sensors: accel
 * +/-8g, gyro +/-500dps, mag +/-4gauss, all continuous-conversion at their
 * highest common practical ODR. Returns ESP_ERR_NOT_FOUND if either
 * WHO_AM_I check fails. */
esp_err_t lsm9ds1_init(lsm9ds1_t *dev, i2c_master_bus_handle_t bus, const lsm9ds1_config_t *cfg);

/* Blocking read of accel+gyro+temp. Does not touch the magnetometer (mag
 * has a much lower ODR -- poll it separately at a slower rate, see
 * lsm9ds1_read_mag). */
esp_err_t lsm9ds1_read_ag(lsm9ds1_t *dev, fc_imu_sample_t *out);

/* Blocking read of the magnetometer only; fills mag_ut in *out, leaves
 * other fields untouched -- caller merges it into the next fc_imu_sample_t. */
esp_err_t lsm9ds1_read_mag(lsm9ds1_t *dev, float mag_ut[3]);

#ifdef __cplusplus
}
#endif
