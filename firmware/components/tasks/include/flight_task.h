#pragma once

#include "driver/gpio.h"
#include "fc/flight_state.h"
#include "queues.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    fc_queues_t *queues;
    gpio_num_t drogue_pyro_gpio;
    gpio_num_t main_pyro_gpio;
    gpio_num_t arm_switch_gpio; /* active-high; pull-down expected externally or via config */
    fc_flight_state_config_t state_cfg; /* thresholds -- see fc_flight_state_config_default() */
} flight_task_config_t;

/* FreeRTOS task entry point, highest priority of the five tasks (see
 * docs/ARCHITECTURE.md task table): deployment must never be delayed by a
 * slow logger or telemetry send. Consumes state_to_flight, drives the
 * fc_flight_state_t machine, fires pyro GPIOs, and forwards phase changes
 * to phase_change_to_logger for the flight log. */
void flight_task(void *pv);

#ifdef __cplusplus
}
#endif
