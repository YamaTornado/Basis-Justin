#include "flight_task.h"

#include <Arduino.h>

/* Pulse width for the pyro MOSFET/driver gate. This is a starting point,
 * not a calibrated value -- tune against your actual igniter/driver
 * circuit. Fired synchronously from flight_task; briefly blocking here is
 * acceptable (deployment is a one-shot event), but keep it short so the
 * task returns to monitoring state updates quickly. */
#define PYRO_FIRE_PULSE_MS 250

struct PyroGpioCtx {
    int drogue_gpio;
    int main_gpio;
};

static void pyro_fire_gpio(fc_pyro_channel_t channel, void *user_ctx) {
    PyroGpioCtx *ctx = (PyroGpioCtx *)user_ctx;
    int pin = (channel == FC_PYRO_DROGUE) ? ctx->drogue_gpio : ctx->main_gpio;
    log_w("flight_task: FIRING pyro channel %d on GPIO %d", channel, pin);
    digitalWrite(pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(PYRO_FIRE_PULSE_MS));
    digitalWrite(pin, LOW);
}

void flight_task(void *pv) {
    FlightTaskConfig *cfg = (FlightTaskConfig *)pv;

    pinMode(cfg->drogue_pyro_gpio, OUTPUT);
    pinMode(cfg->main_pyro_gpio, OUTPUT);
    digitalWrite(cfg->drogue_pyro_gpio, LOW);
    digitalWrite(cfg->main_pyro_gpio, LOW);
    pinMode(cfg->arm_switch_gpio, INPUT_PULLDOWN);

    static PyroGpioCtx pyro_ctx; /* static: must outlive the task, referenced by callback */
    pyro_ctx.drogue_gpio = cfg->drogue_pyro_gpio;
    pyro_ctx.main_gpio = cfg->main_pyro_gpio;

    cfg->state_cfg.pyro_fire = pyro_fire_gpio;
    cfg->state_cfg.pyro_ctx = &pyro_ctx;

    fc_flight_state_t state;
    fc_flight_state_init(&state, &cfg->state_cfg);

    fc_state_estimate_t s;
    for (;;) {
        if (xQueueReceive(cfg->queues->state_to_flight, &s, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (state.phase == FC_PHASE_PAD && digitalRead(cfg->arm_switch_gpio) == HIGH) {
            fc_flight_state_arm(&state, s.timestamp_us);
        }

        fc_flight_phase_t prev_phase = state.phase;
        fc_flight_state_update(&state, &s);
        xQueueOverwrite(cfg->queues->phase_to_telemetry, &state.phase);

        if (state.phase != prev_phase) {
            log_i("flight_task: phase %s -> %s", fc_flight_phase_name(prev_phase),
                  fc_flight_phase_name(state.phase));

            fc_log_phase_change_t change;
            change.timestamp_us = s.timestamp_us;
            change.old_phase = (int8_t)prev_phase;
            change.new_phase = (int8_t)state.phase;
            xQueueSend(cfg->queues->phase_change_to_logger, &change, 0);
        }
    }
}
