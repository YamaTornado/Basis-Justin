#pragma once

#include <Adafruit_LSM9DS1.h>
#include <Wire.h>

#include "fc/types.h"

/* Thin wrapper around Adafruit's LSM9DS1 library: owns the sensor object,
 * exposes fc_imu_sample_t directly so the rest of the firmware never
 * touches Adafruit_LSM9DS1 types. Unit conversion (the library's unified
 * sensor events are in m/s^2 and rad/s) happens here, at the boundary --
 * core/ (fc/types.h) expects g and deg/s (see attitude_filter.h).
 *
 * Uses Wire1 (the ESP32-S3's second I2C controller), kept deliberately
 * separate from the BMP280's bus (Wire) -- call Wire1.begin(sda, scl) in
 * main.cpp before ImuLsm9ds1::begin(). The Adafruit_LSM9DS1 constructor
 * picks the bus, so it's fixed here rather than passed into begin(). */
class ImuLsm9ds1 {
   public:
    bool begin();

    /* Reads accel+gyro+mag+temp in one shot (the library samples all three
     * sub-sensors together) and fills out. Caller stamps timestamp_us
     * (see sensor_task.cpp). Returns false immediately (without touching
     * the bus) if begin() never succeeded -- hammering a sensor that isn't
     * there with I2C transactions every cycle at 100Hz has been observed
     * to eventually crash rather than just fail cleanly. */
    bool read(fc_imu_sample_t *out);

   private:
    Adafruit_LSM9DS1 lsm_{&Wire1};
    bool began_ = false;
};
