#pragma once

#include "bmp280.h"
#include "gps_nmea.h"
#include "lsm9ds1.h"
#include "queues.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    lsm9ds1_t *imu;
    bmp280_t *baro;
    gps_nmea_t *gps;
    fc_queues_t *queues;
    int rate_hz; /* target loop rate, see docs/ARCHITECTURE.md task table */
} sensor_task_config_t;

/* FreeRTOS task entry point. pv must point to a sensor_task_config_t that
 * outlives the task (a static/global instance is fine -- there is exactly
 * one sensor_task). Reads IMU accel/gyro every cycle, baro every cycle,
 * mag and GPS at their own slower natural rates (magnetometer ODR ~40Hz,
 * GPS ~1-10Hz), and pushes tagged raw_sample_t's onto both
 * queues->raw_to_estimator and queues->raw_to_logger (non-blocking; drops
 * under sustained overload rather than ever blocking on a slow consumer). */
void sensor_task(void *pv);

#ifdef __cplusplus
}
#endif
