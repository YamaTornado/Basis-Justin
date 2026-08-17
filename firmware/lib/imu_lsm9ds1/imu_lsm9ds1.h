#pragma once

#include <Adafruit_LSM9DS1.h>

#include "fc/types.h"

/* Thin wrapper around Adafruit's LSM9DS1 library: owns the sensor object,
 * exposes fc_imu_sample_t directly so the rest of the firmware never
 * touches Adafruit_LSM9DS1 types. Unit conversion (the library's unified
 * sensor events are in m/s^2 and rad/s) happens here, at the boundary --
 * core/ (fc/types.h) expects g and deg/s (see attitude_filter.h).
 *
 * Uses the global Wire object -- call Wire.begin(sda, scl) in main.cpp
 * before ImuLsm9ds1::begin(). */
class ImuLsm9ds1 {
   public:
    bool begin();

    /* Reads accel+gyro+mag+temp in one shot (the library samples all three
     * sub-sensors together) and fills out. Caller stamps timestamp_us
     * (see sensor_task.cpp). */
    bool read(fc_imu_sample_t *out);

   private:
    Adafruit_LSM9DS1 lsm_;
};
