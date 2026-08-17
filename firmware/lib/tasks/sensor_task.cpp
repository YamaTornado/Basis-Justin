#include "sensor_task.h"

#include <Arduino.h>
#include <esp_timer.h>

static void push_sample(FcQueues *q, const RawSample *sample) {
    /* Non-blocking on both queues: sensing must never stall waiting for a
     * slow consumer. Dropping a sample under overload is preferable to
     * missing a deadline on the next one. */
    if (xQueueSend(q->raw_to_estimator, sample, 0) != pdTRUE) {
        log_w("sensor_task: raw_to_estimator full, dropping sample (type=%d)", sample->type);
    }
    if (xQueueSend(q->raw_to_logger, sample, 0) != pdTRUE) {
        log_w("sensor_task: raw_to_logger full, dropping sample (type=%d)", sample->type);
    }
}

void sensor_task(void *pv) {
    SensorTaskConfig *cfg = (SensorTaskConfig *)pv;
    TickType_t period_ticks = pdMS_TO_TICKS(1000 / cfg->rate_hz);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        RawSample sample;

        sample.type = RAW_SAMPLE_IMU;
        if (cfg->imu->read(&sample.as.imu)) {
            sample.as.imu.timestamp_us = (int64_t)esp_timer_get_time();
            push_sample(cfg->queues, &sample);
        }

        sample.type = RAW_SAMPLE_BARO;
        if (cfg->baro->read(&sample.as.baro)) {
            sample.as.baro.timestamp_us = (int64_t)esp_timer_get_time();
            push_sample(cfg->queues, &sample);
        }

        sample.type = RAW_SAMPLE_GPS;
        if (cfg->gps->poll(&sample.as.gps)) {
            push_sample(cfg->queues, &sample);
            xQueueOverwrite(cfg->queues->gps_to_telemetry, &sample.as.gps);
        }

        vTaskDelayUntil(&last_wake, period_ticks);
    }
}
