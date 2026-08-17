#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Shared I2C master bus for the two I2C sensors (LSM9DS1, BMP280). Both
 * drivers add their own i2c_master_dev_handle_t to this bus rather than
 * each owning a port. */

typedef struct {
    gpio_num_t sda;
    gpio_num_t scl;
    uint32_t clk_speed_hz; /* 400000 is a safe default for both sensors */
} i2c_bus_config_t;

esp_err_t i2c_bus_init(const i2c_bus_config_t *cfg, i2c_master_bus_handle_t *out_bus);

#ifdef __cplusplus
}
#endif
