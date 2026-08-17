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

/* Bring-up aid: the normal data path (queues -> estimator -> filters)
 * never prints a raw value anywhere, which makes "is this sensor actually
 * wired right" hard to answer just from the serial monitor. Print raw
 * readings at a throttled ~5 Hz (every 20th cycle @ 100 Hz) instead of
 * every sample, so it stays readable. Remove/lower this once bring-up is
 * done and the full pipeline is being exercised instead. */
#define DEBUG_PRINT_DIVIDER 20

void sensor_task(void *pv) {
    SensorTaskConfig *cfg = (SensorTaskConfig *)pv;
    TickType_t period_ticks = pdMS_TO_TICKS(1000 / cfg->rate_hz);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t loop_count = 0;
    bool print_this_cycle;

    for (;;) {
        print_this_cycle = (loop_count % DEBUG_PRINT_DIVIDER) == 0;
        RawSample sample;

        sample.type = RAW_SAMPLE_IMU;
        if (cfg->imu->read(&sample.as.imu)) {
            sample.as.imu.timestamp_us = (int64_t)esp_timer_get_time();
            push_sample(cfg->queues, &sample);
            if (print_this_cycle) {
                const fc_imu_sample_t &imu = sample.as.imu;
                Serial.printf("[IMU]  accel=%.2f,%.2f,%.2fg  gyro=%.1f,%.1f,%.1fdps  "
                              "mag=%.1f,%.1f,%.1fuT  temp=%.1fC\n",
                              imu.accel_g[0], imu.accel_g[1], imu.accel_g[2], imu.gyro_dps[0],
                              imu.gyro_dps[1], imu.gyro_dps[2], imu.mag_ut[0], imu.mag_ut[1],
                              imu.mag_ut[2], imu.temp_c);
            }
        } else if (print_this_cycle) {
            Serial.println("[IMU]  read failed -- check wiring/I2C address");
        }

        sample.type = RAW_SAMPLE_BARO;
        if (cfg->baro->read(&sample.as.baro)) {
            sample.as.baro.timestamp_us = (int64_t)esp_timer_get_time();
            push_sample(cfg->queues, &sample);
            if (print_this_cycle) {
                Serial.printf("[BARO] pressure=%.1fPa  temp=%.1fC\n", sample.as.baro.pressure_pa,
                              sample.as.baro.temperature_c);
            }
        } else if (print_this_cycle) {
            Serial.println("[BARO] read failed -- check wiring/I2C address");
        }

        sample.type = RAW_SAMPLE_GPS;
        if (cfg->gps->poll(&sample.as.gps)) {
            push_sample(cfg->queues, &sample);
            xQueueOverwrite(cfg->queues->gps_to_telemetry, &sample.as.gps);
            const fc_gps_sample_t &gps = sample.as.gps;
            Serial.printf("[GPS]  fix=%u sats=%u lat=%.6f lon=%.6f alt=%.1fm\n", gps.fix_type,
                          gps.num_sats, gps.latitude_deg, gps.longitude_deg, gps.altitude_m);
        }

        loop_count++;
        vTaskDelayUntil(&last_wake, period_ticks);
    }
}
