#pragma once

#include "esp_err.h"
#include "fc/log_format.h"
#include "fc/types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RAW_SAMPLE_IMU,
    RAW_SAMPLE_BARO,
    RAW_SAMPLE_GPS,
} raw_sample_type_t;

typedef struct {
    raw_sample_type_t type;
    union {
        fc_imu_sample_t imu;
        fc_baro_sample_t baro;
        fc_gps_sample_t gps;
    } as;
} raw_sample_t;

/* Fan-out queues, see docs/ARCHITECTURE.md "Datenfluss (Firmware)":
 * sensor_task writes raw samples to both estimator and logger; estimator
 * writes the fused state to flight/logger/telemetry. Using separate queues
 * per consumer (rather than one queue + broadcast) means a slow consumer
 * (telemetry over LoRa) can never block a fast/critical one (flight_task).
 *
 * state_to_* are length-1 "mailbox" queues written with xQueueOverwrite:
 * consumers only ever care about the *latest* state, never a backlog. */
typedef struct {
    QueueHandle_t raw_to_estimator; /* depth ~64, may drop under sustained overload */
    QueueHandle_t raw_to_logger;    /* depth ~64, may drop under sustained overload */
    QueueHandle_t state_to_flight;    /* depth 1, xQueueOverwrite */
    QueueHandle_t state_to_logger;    /* depth 1, xQueueOverwrite */
    QueueHandle_t state_to_telemetry; /* depth 1, xQueueOverwrite */
    QueueHandle_t phase_change_to_logger; /* depth 8, one entry per transition, rare */
    QueueHandle_t gps_to_telemetry; /* depth 1, xQueueOverwrite -- latest fix for telemetry_task,
                                      * since fc_state_estimate_t carries no lat/lon */
    QueueHandle_t phase_to_telemetry; /* depth 1, xQueueOverwrite -- latest fc_flight_phase_t,
                                        * written by flight_task, read by telemetry_task */
} fc_queues_t;

esp_err_t fc_queues_create(fc_queues_t *q);

#ifdef __cplusplus
}
#endif
