#include "sensor_task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

static const char *TAG = "sensor_task";

/* Read the magnetometer at ~1/3 of the main loop rate -- its ODR (~40Hz,
 * see lsm9ds1.c) is far below the 100Hz accel/gyro/baro rate, no point
 * hammering it every cycle. */
#define MAG_READ_DIVIDER 3

static void push_sample(fc_queues_t *q, const raw_sample_t *sample) {
    /* Non-blocking on both queues: sensing must never stall waiting for a
     * slow consumer. Dropping a sample under overload is preferable to
     * missing a deadline on the next one. */
    if (xQueueSend(q->raw_to_estimator, sample, 0) != pdTRUE) {
        ESP_LOGW(TAG, "raw_to_estimator full, dropping sample (type=%d)", sample->type);
    }
    if (xQueueSend(q->raw_to_logger, sample, 0) != pdTRUE) {
        ESP_LOGW(TAG, "raw_to_logger full, dropping sample (type=%d)", sample->type);
    }
}

void sensor_task(void *pv) {
    sensor_task_config_t *cfg = (sensor_task_config_t *)pv;
    TickType_t period_ticks = pdMS_TO_TICKS(1000 / cfg->rate_hz);
    TickType_t last_wake = xTaskGetTickCount();

    float last_mag_ut[3] = {0.0f, 0.0f, 0.0f};
    uint32_t loop_count = 0;

    for (;;) {
        raw_sample_t sample;

        if (loop_count % MAG_READ_DIVIDER == 0) {
            lsm9ds1_read_mag(cfg->imu, last_mag_ut);
        }

        sample.type = RAW_SAMPLE_IMU;
        if (lsm9ds1_read_ag(cfg->imu, &sample.as.imu) == ESP_OK) {
            sample.as.imu.timestamp_us = esp_timer_get_time();
            sample.as.imu.mag_ut[0] = last_mag_ut[0];
            sample.as.imu.mag_ut[1] = last_mag_ut[1];
            sample.as.imu.mag_ut[2] = last_mag_ut[2];
            push_sample(cfg->queues, &sample);
        }

        sample.type = RAW_SAMPLE_BARO;
        if (bmp280_read(cfg->baro, &sample.as.baro) == ESP_OK) {
            sample.as.baro.timestamp_us = esp_timer_get_time();
            push_sample(cfg->queues, &sample);
        }

        sample.type = RAW_SAMPLE_GPS;
        if (gps_nmea_poll(cfg->gps, &sample.as.gps, 0)) {
            push_sample(cfg->queues, &sample);
            xQueueOverwrite(cfg->queues->gps_to_telemetry, &sample.as.gps);
        }

        loop_count++;
        vTaskDelayUntil(&last_wake, period_ticks);
    }
}
