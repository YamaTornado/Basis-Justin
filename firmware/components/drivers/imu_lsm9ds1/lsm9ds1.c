#include "lsm9ds1.h"

#include "esp_log.h"

static const char *TAG = "lsm9ds1";

/* --- AG (accel + gyro) registers --- */
#define REG_WHO_AM_I_AG 0x0F
#define WHO_AM_I_AG_VALUE 0x68
#define REG_CTRL_REG1_G 0x10
#define REG_CTRL_REG6_XL 0x20
#define REG_CTRL_REG8 0x22
#define REG_OUT_TEMP_L 0x15
#define REG_OUT_X_L_G 0x18
#define REG_OUT_X_L_XL 0x28

/* --- M (magnetometer) registers --- */
#define REG_WHO_AM_I_M 0x0F
#define WHO_AM_I_M_VALUE 0x3D
#define REG_CTRL_REG1_M 0x20
#define REG_CTRL_REG2_M 0x21
#define REG_CTRL_REG3_M 0x22
#define REG_OUT_X_L_M 0x28

/* Sensitivities for the ranges configured in lsm9ds1_init() below. If you
 * change the range bits, update these to match (see LSM9DS1 datasheet
 * table 3/table 78 for the other options). */
#define ACCEL_SENSITIVITY_G_PER_LSB (0.000244f)   /* +/-8g range */
#define GYRO_SENSITIVITY_DPS_PER_LSB (0.01750f)   /* +/-500dps range */
#define MAG_SENSITIVITY_GAUSS_PER_LSB (0.00014f)  /* +/-4gauss range */
#define GAUSS_TO_UT 100.0f

static esp_err_t write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

static esp_err_t read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *out, size_t len) {
    /* LSM9DS1: set bit 7 of the sub-address to auto-increment across a
     * multi-byte burst read. */
    uint8_t addr = (len > 1) ? (uint8_t)(reg | 0x80) : reg;
    return i2c_master_transmit_receive(dev, &addr, 1, out, len, 100);
}

void lsm9ds1_config_default(lsm9ds1_config_t *cfg) {
    cfg->addr_ag = LSM9DS1_ADDR_AG_DEFAULT;
    cfg->addr_m = LSM9DS1_ADDR_M_DEFAULT;
}

esp_err_t lsm9ds1_init(lsm9ds1_t *dev, i2c_master_bus_handle_t bus, const lsm9ds1_config_t *cfg) {
    i2c_device_config_t ag_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = cfg->addr_ag,
        .scl_speed_hz = 400000,
    };
    i2c_device_config_t m_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = cfg->addr_m,
        .scl_speed_hz = 400000,
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &ag_cfg, &dev->ag);
    if (err != ESP_OK) return err;
    err = i2c_master_bus_add_device(bus, &m_cfg, &dev->m);
    if (err != ESP_OK) return err;

    uint8_t who = 0;
    err = read_regs(dev->ag, REG_WHO_AM_I_AG, &who, 1);
    if (err != ESP_OK || who != WHO_AM_I_AG_VALUE) {
        ESP_LOGE(TAG, "AG WHO_AM_I mismatch: got 0x%02X, expected 0x%02X (err=%d)", who,
                  WHO_AM_I_AG_VALUE, err);
        return ESP_ERR_NOT_FOUND;
    }
    err = read_regs(dev->m, REG_WHO_AM_I_M, &who, 1);
    if (err != ESP_OK || who != WHO_AM_I_M_VALUE) {
        ESP_LOGE(TAG, "M WHO_AM_I mismatch: got 0x%02X, expected 0x%02X (err=%d)", who,
                  WHO_AM_I_M_VALUE, err);
        return ESP_ERR_NOT_FOUND;
    }

    /* Gyro: ODR 119 Hz, +/-500 dps (CTRL_REG1_G: ODR_G[2:0]=011, FS_G[1:0]=01) */
    esp_err_t e1 = write_reg(dev->ag, REG_CTRL_REG1_G, 0x68);
    /* Accel: ODR 119 Hz, +/-8g (CTRL_REG6_XL: ODR_XL[2:0]=011, FS_XL[1:0]=11) */
    esp_err_t e2 = write_reg(dev->ag, REG_CTRL_REG6_XL, 0x6C);
    /* BDU=1 (block data update, avoids torn reads across L/H bytes),
     * IF_ADD_INC=1 (already default, set explicitly) */
    esp_err_t e3 = write_reg(dev->ag, REG_CTRL_REG8, 0x44);

    /* Mag: ultra-high performance on X/Y, ODR 40 Hz (CTRL_REG1_M) */
    esp_err_t e4 = write_reg(dev->m, REG_CTRL_REG1_M, 0x7C);
    /* +/-4 gauss range (CTRL_REG2_M: FS[1:0]=00) */
    esp_err_t e5 = write_reg(dev->m, REG_CTRL_REG2_M, 0x00);
    /* Continuous-conversion mode (CTRL_REG3_M: MD[1:0]=00) */
    esp_err_t e6 = write_reg(dev->m, REG_CTRL_REG3_M, 0x00);

    if (e1 != ESP_OK) return e1;
    if (e2 != ESP_OK) return e2;
    if (e3 != ESP_OK) return e3;
    if (e4 != ESP_OK) return e4;
    if (e5 != ESP_OK) return e5;
    if (e6 != ESP_OK) return e6;
    return ESP_OK;
}

static int16_t le16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

esp_err_t lsm9ds1_read_ag(lsm9ds1_t *dev, fc_imu_sample_t *out) {
    /* OUT_TEMP_L (0x15) and the gyro/accel outputs (0x18-0x2D) are not
     * contiguous, so read them in two bursts instead of one. */
    uint8_t temp_buf[2];
    esp_err_t err = read_regs(dev->ag, REG_OUT_TEMP_L, temp_buf, 2);
    if (err != ESP_OK) return err;
    int16_t temp_raw = le16(temp_buf);

    uint8_t gx[12];
    err = read_regs(dev->ag, REG_OUT_X_L_G, gx, 12); /* gyro(6) then accel(6), contiguous */
    if (err != ESP_OK) return err;

    out->gyro_dps[0] = le16(&gx[0]) * GYRO_SENSITIVITY_DPS_PER_LSB;
    out->gyro_dps[1] = le16(&gx[2]) * GYRO_SENSITIVITY_DPS_PER_LSB;
    out->gyro_dps[2] = le16(&gx[4]) * GYRO_SENSITIVITY_DPS_PER_LSB;
    out->accel_g[0] = le16(&gx[6]) * ACCEL_SENSITIVITY_G_PER_LSB;
    out->accel_g[1] = le16(&gx[8]) * ACCEL_SENSITIVITY_G_PER_LSB;
    out->accel_g[2] = le16(&gx[10]) * ACCEL_SENSITIVITY_G_PER_LSB;
    /* Datasheet: T_degC = 25 + TEMP_OUT / 16 (12-bit, left-justified in 16 bits) */
    out->temp_c = 25.0f + (float)temp_raw / 16.0f;

    return ESP_OK;
}

esp_err_t lsm9ds1_read_mag(lsm9ds1_t *dev, float mag_ut[3]) {
    uint8_t buf[6];
    esp_err_t err = read_regs(dev->m, REG_OUT_X_L_M, buf, 6);
    if (err != ESP_OK) return err;

    mag_ut[0] = le16(&buf[0]) * MAG_SENSITIVITY_GAUSS_PER_LSB * GAUSS_TO_UT;
    mag_ut[1] = le16(&buf[2]) * MAG_SENSITIVITY_GAUSS_PER_LSB * GAUSS_TO_UT;
    mag_ut[2] = le16(&buf[4]) * MAG_SENSITIVITY_GAUSS_PER_LSB * GAUSS_TO_UT;
    return ESP_OK;
}
