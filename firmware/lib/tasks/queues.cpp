#include "queues.h"

#define RAW_QUEUE_DEPTH 64

bool fc_queues_create(FcQueues *q) {
    q->raw_to_estimator = xQueueCreate(RAW_QUEUE_DEPTH, sizeof(RawSample));
    q->raw_to_logger = xQueueCreate(RAW_QUEUE_DEPTH, sizeof(RawSample));
    q->state_to_flight = xQueueCreate(1, sizeof(fc_state_estimate_t));
    q->state_to_logger = xQueueCreate(1, sizeof(fc_state_estimate_t));
    q->state_to_telemetry = xQueueCreate(1, sizeof(fc_state_estimate_t));
    q->phase_change_to_logger = xQueueCreate(8, sizeof(fc_log_phase_change_t));
    q->gps_to_telemetry = xQueueCreate(1, sizeof(fc_gps_sample_t));
    q->phase_to_telemetry = xQueueCreate(1, sizeof(fc_flight_phase_t));

    return q->raw_to_estimator && q->raw_to_logger && q->state_to_flight && q->state_to_logger &&
           q->state_to_telemetry && q->phase_change_to_logger && q->gps_to_telemetry &&
           q->phase_to_telemetry;
}
