#include "flight_task.h"

#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "flight_task";

/* Pulse width for the pyro MOSFET/driver gate. This is a starting point,
 * not a calibrated value -- tune against your actual igniter/driver
 * circuit. Fired synchronously from flight_task; briefly blocking here is
 * acceptable (deployment is a one-shot event), but keep it short so the
 * task returns to monitoring state updates quickly. */
#define PYRO_FIRE_PULSE_MS 250

typedef struct {
    gpio_num_t drogue_gpio;
    gpio_num_t main_gpio;
} pyro_gpio_ctx_t;

static void pyro_fire_gpio(fc_pyro_channel_t channel, void *user_ctx) {
    pyro_gpio_ctx_t *ctx = (pyro_gpio_ctx_t *)user_ctx;
    gpio_num_t pin = (channel == FC_PYRO_DROGUE) ? ctx->drogue_gpio : ctx->main_gpio;
    ESP_LOGW(TAG, "FIRING pyro channel %d on GPIO %d", channel, pin);
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(PYRO_FIRE_PULSE_MS));
    gpio_set_level(pin, 0);
}

void flight_task(void *pv) {
    flight_task_config_t *cfg = (flight_task_config_t *)pv;

    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << cfg->drogue_pyro_gpio) | (1ULL << cfg->main_pyro_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);
    gpio_set_level(cfg->drogue_pyro_gpio, 0);
    gpio_set_level(cfg->main_pyro_gpio, 0);

    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << cfg->arm_switch_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_cfg);

    static pyro_gpio_ctx_t pyro_ctx; /* static: must outlive the task, referenced by callback */
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

        if (state.phase == FC_PHASE_PAD && gpio_get_level(cfg->arm_switch_gpio)) {
            fc_flight_state_arm(&state, s.timestamp_us);
        }

        fc_flight_phase_t prev_phase = state.phase;
        fc_flight_state_update(&state, &s);
        xQueueOverwrite(cfg->queues->phase_to_telemetry, &state.phase);

        if (state.phase != prev_phase) {
            ESP_LOGI(TAG, "phase: %s -> %s", fc_flight_phase_name(prev_phase),
                      fc_flight_phase_name(state.phase));

            fc_log_phase_change_t change = {
                .timestamp_us = s.timestamp_us,
                .old_phase = (int8_t)prev_phase,
                .new_phase = (int8_t)state.phase,
            };
            xQueueSend(cfg->queues->phase_change_to_logger, &change, 0);
        }
    }
}
