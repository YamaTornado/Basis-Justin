#include "i2c_bus.h"

esp_err_t i2c_bus_init(const i2c_bus_config_t *cfg, i2c_master_bus_handle_t *out_bus) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = cfg->sda,
        .scl_io_num = cfg->scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_cfg, out_bus);
}
