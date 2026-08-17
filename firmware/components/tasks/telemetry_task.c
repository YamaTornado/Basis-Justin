#include "telemetry_task.h"

#include <stdbool.h>

#include "esp_log.h"
#include "fc/telemetry.h"
#include "freertos/task.h"

static const char *TAG = "telemetry_task";

void telemetry_task(void *pv) {
    telemetry_task_config_t *cfg = (telemetry_task_config_t *)pv;
    TickType_t period_ticks = pdMS_TO_TICKS(1000 / cfg->rate_hz);
    TickType_t last_wake = xTaskGetTickCount();

    uint16_t seq = 0;

    for (;;) {
        fc_state_estimate_t state = {0};
        fc_gps_sample_t gps = {0};
        fc_flight_phase_t phase = FC_PHASE_PAD;

        bool have_state = xQueuePeek(cfg->queues->state_to_telemetry, &state, 0) == pdTRUE;
        xQueuePeek(cfg->queues->gps_to_telemetry, &gps, 0);
        xQueuePeek(cfg->queues->phase_to_telemetry, &phase, 0);

        if (have_state) {
            fc_telemetry_status_t status = {0};
            status.seq = seq++;
            status.timestamp_us = state.timestamp_us;
            status.phase = (int8_t)phase;
            status.altitude_m = state.altitude_m;
            status.velocity_mps = state.velocity_mps;
            status.latitude_deg = gps.latitude_deg;
            status.longitude_deg = gps.longitude_deg;
            status.gps_fix_type = gps.fix_type;
            status.num_sats = gps.num_sats;
            status.battery_pct = 0xFF; /* TODO: wire up a battery ADC reading */

            uint8_t frame[FC_TLM_MAX_FRAME];
            size_t n = fc_telemetry_encode_status(&status, frame, sizeof(frame));
            if (n > 0) {
                esp_err_t err = lora_lr02_send(cfg->lora, frame, n);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "telemetry send failed: %d", err);
                }
            }
        }

        vTaskDelayUntil(&last_wake, period_ticks);
    }
}
