#pragma once

#include "baro_bmp280.h"
#include "gps_m100.h"
#include "imu_lsm9ds1.h"
#include "queues.h"

struct SensorTaskConfig {
    ImuLsm9ds1 *imu;
    BaroBmp280 *baro;
    GpsM100 *gps;
    FcQueues *queues;
    int rate_hz; /* target loop rate, see docs/ARCHITECTURE.md task table */
};

/* FreeRTOS task entry point. pv must point to a SensorTaskConfig that
 * outlives the task (a static/global instance is fine -- there is exactly
 * one sensor_task). Reads IMU accel/gyro/mag and baro every cycle, GPS at
 * its own slower natural rate, and pushes tagged RawSample's onto both
 * queues->raw_to_estimator and queues->raw_to_logger (non-blocking; drops
 * under sustained overload rather than ever blocking on a slow consumer). */
void sensor_task(void *pv);
