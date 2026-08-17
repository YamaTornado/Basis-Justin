#pragma once

#include "queues.h"

struct EstimatorTaskConfig {
    FcQueues *queues;
};

/* FreeRTOS task entry point. Consumes queues->raw_to_estimator, runs the
 * attitude filter (Madgwick) on every IMU sample and the altitude filter
 * (linear KF) predict/update on every IMU/baro sample respectively, and
 * publishes the resulting fc_state_estimate_t to state_to_{flight,logger,
 * telemetry} via xQueueOverwrite. See docs/ARCHITECTURE.md "Estimation". */
void estimator_task(void *pv);
