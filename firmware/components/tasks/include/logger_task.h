#pragma once

#include "flash_log.h"
#include "queues.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    fc_queues_t *queues;
    flash_log_t *log; /* already initialized, see flash_log_init() */
    int rate_hz;       /* how often state_to_logger is sampled; raw samples and
                         * phase changes are drained every cycle regardless */
} logger_task_config_t;

/* FreeRTOS task entry point. Drains raw_to_logger and phase_change_to_logger
 * every cycle (never lets them build a backlog), and samples state_to_logger
 * once per cycle -- all written as fc_log_format frames to flash_log. */
void logger_task(void *pv);

#ifdef __cplusplus
}
#endif
