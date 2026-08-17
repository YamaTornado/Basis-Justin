#include "bmp280.h"

#include <math.h>

#include "esp_log.h"

static const char *TAG = "bmp280";

#define REG_CALIB_START 0x88
#define REG_CHIP_ID 0xD0
#define CHIP_ID_VALUE 0x58
#define REG_RESET 0xE0
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG 0xF5
#define REG_PRESS_MSB 0xF7

static esp_err_t write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

static esp_err_t read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *out, size_t len) {
    return i2c_master_transmit_receive(dev, &reg, 1, out, len, 100);
}

static uint16_t le_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static int16_t le_s16(const uint8_t *p) {
    return (int16_t)le_u16(p);
}

esp_err_t bmp280_init(bmp280_t *dev, i2c_master_bus_handle_t bus, uint16_t addr) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dev->dev);
    if (err != ESP_OK) return err;

    uint8_t chip_id = 0;
    err = read_regs(dev->dev, REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK || chip_id != CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "chip id mismatch: got 0x%02X, expected 0x%02X (err=%d)", chip_id,
                  CHIP_ID_VALUE, err);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t calib[24];
    err = read_regs(dev->dev, REG_CALIB_START, calib, sizeof(calib));
    if (err != ESP_OK) return err;

    dev->dig_T1 = le_u16(&calib[0]);
    dev->dig_T2 = le_s16(&calib[2]);
    dev->dig_T3 = le_s16(&calib[4]);
    dev->dig_P1 = le_u16(&calib[6]);
    dev->dig_P2 = le_s16(&calib[8]);
    dev->dig_P3 = le_s16(&calib[10]);
    dev->dig_P4 = le_s16(&calib[12]);
    dev->dig_P5 = le_s16(&calib[14]);
    dev->dig_P6 = le_s16(&calib[16]);
    dev->dig_P7 = le_s16(&calib[18]);
    dev->dig_P8 = le_s16(&calib[20]);
    dev->dig_P9 = le_s16(&calib[22]);

    /* ctrl_meas: osrs_t=x1 (001), osrs_p=x4 (011), mode=normal (11) -> good
     * default for an altimeter (pressure resolution matters more than
     * temperature). config: t_sb=0.5ms (000), filter=x4 (010). */
    err = write_reg(dev->dev, REG_CONFIG, 0x08);
    if (err != ESP_OK) return err;
    err = write_reg(dev->dev, REG_CTRL_MEAS, 0x2F);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

/* Bosch reference compensation formulas (BMP280 datasheet 3.11.3, double
 * precision variant) -- kept as close to the datasheet as practical so
 * they're easy to cross-check. */
esp_err_t bmp280_read(bmp280_t *dev, fc_baro_sample_t *out) {
    uint8_t buf[6];
    esp_err_t err = read_regs(dev->dev, REG_PRESS_MSB, buf, sizeof(buf));
    if (err != ESP_OK) return err;

    int32_t adc_p = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
    int32_t adc_t = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);

    double var1, var2, t_fine, temperature, pressure;

    var1 = (((double)adc_t) / 16384.0 - ((double)dev->dig_T1) / 1024.0) * ((double)dev->dig_T2);
    var2 = ((((double)adc_t) / 131072.0 - ((double)dev->dig_T1) / 8192.0) *
            (((double)adc_t) / 131072.0 - ((double)dev->dig_T1) / 8192.0)) *
           ((double)dev->dig_T3);
    t_fine = var1 + var2;
    temperature = t_fine / 5120.0;

    var1 = (t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)dev->dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)dev->dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)dev->dig_P4) * 65536.0);
    var1 = (((double)dev->dig_P3) * var1 * var1 / 524288.0 + ((double)dev->dig_P2) * var1) /
           524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)dev->dig_P1);
    if (var1 == 0.0) {
        out->pressure_pa = 0.0f;
        out->temperature_c = (float)temperature;
        return ESP_OK; /* avoid div by zero, matches datasheet reference code */
    }
    pressure = 1048576.0 - (double)adc_p;
    pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)dev->dig_P9) * pressure * pressure / 2147483648.0;
    var2 = pressure * ((double)dev->dig_P8) / 32768.0;
    pressure = pressure + (var1 + var2 + ((double)dev->dig_P7)) / 16.0;

    out->pressure_pa = (float)pressure;
    out->temperature_c = (float)temperature;
    return ESP_OK;
}

float bmp280_pressure_to_altitude_m(float pressure_pa, float reference_pressure_pa) {
    /* International barometric formula, valid for the troposphere -- more
     * than enough range for a hobby rocket flight. */
    return 44330.0f * (1.0f - powf(pressure_pa / reference_pressure_pa, 0.1903f));
}
