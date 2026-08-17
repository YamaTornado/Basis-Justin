#pragma once

#include "lora_lr02.h"
#include "queues.h"

struct TelemetryTaskConfig {
    FcQueues *queues;
    LoraLr02 *lora; /* already begin()'d */
    int rate_hz;      /* 5-10 Hz suggested, see docs/ARCHITECTURE.md task table */
};

/* FreeRTOS task entry point, lowest priority of the five tasks: a slow or
 * stalled LoRa send must never affect sensing/estimation/deployment/logging.
 * Samples state_to_telemetry + gps_to_telemetry mailboxes at rate_hz and
 * sends one fc_telemetry_status_t packet per cycle. */
void telemetry_task(void *pv);
