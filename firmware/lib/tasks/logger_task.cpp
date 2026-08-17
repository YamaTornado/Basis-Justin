#include "logger_task.h"

#include <Arduino.h>

#include "fc/log_format.h"

static void write_frame(FlashLog *log, const uint8_t *frame, size_t n) {
    if (n == 0) {
        return;
    }
    if (!log->write(frame, n)) {
        static bool warned = false;
        if (!warned) {
            log_e("logger_task: flight_log partition full -- further records are dropped");
            warned = true;
        }
    }
}

static void drain_raw_queue(LoggerTaskConfig *cfg) {
    RawSample sample;
    uint8_t frame[FC_LOG_MAX_FRAME];

    while (xQueueReceive(cfg->queues->raw_to_logger, &sample, 0) == pdTRUE) {
        size_t n = 0;
        switch (sample.type) {
            case RAW_SAMPLE_IMU:
                n = fc_log_encode_imu(&sample.as.imu, frame, sizeof(frame));
                break;
            case RAW_SAMPLE_BARO:
                n = fc_log_encode_baro(&sample.as.baro, frame, sizeof(frame));
                break;
            case RAW_SAMPLE_GPS:
                n = fc_log_encode_gps(&sample.as.gps, frame, sizeof(frame));
                break;
        }
        write_frame(cfg->log, frame, n);
    }
}

static void drain_phase_changes(LoggerTaskConfig *cfg) {
    fc_log_phase_change_t change;
    uint8_t frame[FC_LOG_MAX_FRAME];

    while (xQueueReceive(cfg->queues->phase_change_to_logger, &change, 0) == pdTRUE) {
        size_t n = fc_log_encode_phase_change(change.timestamp_us, (fc_flight_phase_t)change.old_phase,
                                               (fc_flight_phase_t)change.new_phase, frame,
                                               sizeof(frame));
        write_frame(cfg->log, frame, n);
    }
}

static void sample_state(LoggerTaskConfig *cfg) {
    fc_state_estimate_t state;
    if (xQueuePeek(cfg->queues->state_to_logger, &state, 0) != pdTRUE) {
        return;
    }
    uint8_t frame[FC_LOG_MAX_FRAME];
    size_t n = fc_log_encode_state(&state, frame, sizeof(frame));
    write_frame(cfg->log, frame, n);
}

void logger_task(void *pv) {
    LoggerTaskConfig *cfg = (LoggerTaskConfig *)pv;
    TickType_t period_ticks = pdMS_TO_TICKS(1000 / cfg->rate_hz);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        drain_raw_queue(cfg);
        drain_phase_changes(cfg);
        sample_state(cfg);

        vTaskDelayUntil(&last_wake, period_ticks);
    }
}
