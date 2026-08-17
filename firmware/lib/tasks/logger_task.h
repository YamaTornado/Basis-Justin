#pragma once

#include "flash_log.h"
#include "queues.h"

struct LoggerTaskConfig {
    FcQueues *queues;
    FlashLog *log; /* already begin()'d */
    int rate_hz;    /* how often state_to_logger is sampled; raw samples and
                      * phase changes are drained every cycle regardless */
};

/* FreeRTOS task entry point. Drains raw_to_logger and phase_change_to_logger
 * every cycle (never lets them build a backlog), and samples state_to_logger
 * once per cycle -- all written as fc_log_format frames to FlashLog. */
void logger_task(void *pv);
