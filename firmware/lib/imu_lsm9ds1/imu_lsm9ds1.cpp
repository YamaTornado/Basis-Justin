#include "imu_lsm9ds1.h"

static constexpr float MPS2_PER_G = 9.80665f;
static constexpr float DPS_PER_RADPS = 57.29578f;

bool ImuLsm9ds1::begin() {
    if (!lsm_.begin()) {
        began_ = false;
        return false;
    }
    /* +/-8g / 500dps / 4gauss: same ranges as the earlier hand-rolled
     * register-level driver used, see docs/ARCHITECTURE.md. */
    lsm_.setupAccel(lsm_.LSM9DS1_ACCELRANGE_8G);
    lsm_.setupGyro(lsm_.LSM9DS1_GYROSCALE_500DPS);
    lsm_.setupMag(lsm_.LSM9DS1_MAGGAIN_4GAUSS);
    began_ = true;
    return true;
}

bool ImuLsm9ds1::read(fc_imu_sample_t *out) {
    if (!began_) {
        return false;
    }
    sensors_event_t accel, mag, gyro, temp;
    lsm_.getEvent(&accel, &mag, &gyro, &temp);

    /* Adafruit unified sensor events are in SI units (m/s^2, rad/s, uT) --
     * convert to the g / deg-per-second convention core/fc/types.h expects. */
    out->accel_g[0] = accel.acceleration.x / MPS2_PER_G;
    out->accel_g[1] = accel.acceleration.y / MPS2_PER_G;
    out->accel_g[2] = accel.acceleration.z / MPS2_PER_G;

    out->gyro_dps[0] = gyro.gyro.x * DPS_PER_RADPS;
    out->gyro_dps[1] = gyro.gyro.y * DPS_PER_RADPS;
    out->gyro_dps[2] = gyro.gyro.z * DPS_PER_RADPS;

    out->mag_ut[0] = mag.magnetic.x;
    out->mag_ut[1] = mag.magnetic.y;
    out->mag_ut[2] = mag.magnetic.z;

    out->temp_c = temp.temperature;
    return true;
}
