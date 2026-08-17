#include "estimator_task.h"

#include <math.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "fc/altitude_filter.h"
#include "fc/attitude_filter.h"
#include "freertos/task.h"

static const char *TAG = "estimator_task";

#define GRAVITY_MPS2 9.80665f
#define MADGWICK_BETA 0.06f
#define REFERENCE_CALIB_SAMPLES 50 /* ~0.5s of baro samples at 100Hz, vehicle stationary on pad */

static void publish_state(fc_queues_t *q, const fc_state_estimate_t *state) {
    xQueueOverwrite(q->state_to_flight, state);
    xQueueOverwrite(q->state_to_logger, state);
    xQueueOverwrite(q->state_to_telemetry, state);
}

void estimator_task(void *pv) {
    estimator_task_config_t *cfg = (estimator_task_config_t *)pv;

    fc_attitude_filter_t attitude;
    fc_attitude_filter_init(&attitude, MADGWICK_BETA, /*have_mag=*/true);

    fc_altitude_filter_t altitude;
    fc_altitude_filter_init(&altitude, 0.0f);

    float reference_pressure_pa = 0.0f;
    float pressure_accum = 0.0f;
    int pressure_count = 0;
    bool have_reference = false;

    int64_t last_imu_us = 0;
    bool have_last_imu = false;

    raw_sample_t sample;
    for (;;) {
        if (xQueueReceive(cfg->queues->raw_to_estimator, &sample, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (sample.type == RAW_SAMPLE_BARO) {
            if (!have_reference) {
                pressure_accum += sample.as.baro.pressure_pa;
                pressure_count++;
                if (pressure_count >= REFERENCE_CALIB_SAMPLES) {
                    reference_pressure_pa = pressure_accum / (float)pressure_count;
                    have_reference = true;
                    ESP_LOGI(TAG, "pad reference pressure: %.1f Pa", reference_pressure_pa);
                }
                continue;
            }
            /* International barometric formula, pad-relative altitude. */
            float baro_alt_m =
                44330.0f * (1.0f - powf(sample.as.baro.pressure_pa / reference_pressure_pa, 0.1903f));
            fc_altitude_filter_update_baro(&altitude, baro_alt_m);
        } else if (sample.type == RAW_SAMPLE_GPS) {
            if (have_reference && sample.as.gps.fix_type > 0) {
                /* GPS altitude is MSL, not pad-relative -- without a
                 * separate pad GPS-altitude calibration this would bias
                 * the filter, so GPS altitude fusion is intentionally left
                 * disabled for now. Position (lat/lon) still flows through
                 * to logging/telemetry via the state estimate consumers
                 * reading raw GPS samples directly. */
            }
            continue;
        } else if (sample.type == RAW_SAMPLE_IMU) {
            if (!have_reference) {
                continue; /* wait for pad pressure calibration before trusting accel integration */
            }

            int64_t now_us = sample.as.imu.timestamp_us;
            float dt_s = have_last_imu ? (float)(now_us - last_imu_us) / 1e6f : 0.01f;
            if (dt_s <= 0.0f || dt_s > 0.5f) {
                dt_s = 0.01f; /* clamp against clock glitches / first sample */
            }
            last_imu_us = now_us;
            have_last_imu = true;

            fc_attitude_filter_update(&attitude, sample.as.imu.gyro_dps, sample.as.imu.accel_g,
                                       sample.as.imu.mag_ut, dt_s);

            /* Rotate specific force into world frame; on the pad this reads
             * ~+1g along the world-up axis (reaction to gravity). Subtract
             * that to get true world-vertical acceleration (0 at rest,
             * positive while thrusting up). Assumes accel convention
             * matches LSM9DS1 datasheet sign (reaction-force positive) --
             * verify against real hardware once assembled. */
            float world_g[3];
            fc_attitude_filter_rotate_to_world(&attitude, sample.as.imu.accel_g, world_g);
            float vertical_accel_mps2 = (world_g[2] - 1.0f) * GRAVITY_MPS2;

            fc_altitude_filter_predict(&altitude, vertical_accel_mps2, dt_s);

            fc_state_estimate_t state = {0};
            state.timestamp_us = now_us;
            state.quat[0] = attitude.q[0];
            state.quat[1] = attitude.q[1];
            state.quat[2] = attitude.q[2];
            state.quat[3] = attitude.q[3];
            state.altitude_m = altitude.x[0];
            state.velocity_mps = altitude.x[1];
            state.accel_bias_mps2 = altitude.x[2];
            state.vertical_accel_mps2 = vertical_accel_mps2;

            publish_state(cfg->queues, &state);
        }
    }
}
