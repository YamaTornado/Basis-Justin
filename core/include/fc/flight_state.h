#pragma once

#include <stdint.h>

#include "fc/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FC_PYRO_DROGUE = 0,
    FC_PYRO_MAIN = 1,
    FC_PYRO_CHANNEL_COUNT
} fc_pyro_channel_t;

/* Registered by the firmware layer at init; core/ never touches a GPIO
 * directly so the state machine stays hardware-free and unit-testable. */
typedef void (*fc_pyro_fire_fn)(fc_pyro_channel_t channel, void *user_ctx);

typedef struct {
    /* vertical accel (m/s^2) sustained for boost_debounce_samples -> BOOST */
    float boost_accel_threshold_mps2;
    int boost_debounce_samples;

    /* velocity sign flip sustained for apogee_debounce_samples -> APOGEE */
    int apogee_debounce_samples;
    /* safety backup: force drogue deploy if apogee not detected within this
     * many ms after boost start */
    uint32_t max_coast_time_ms;

    /* altitude AGL below which main deploys during descent */
    float main_deploy_altitude_m;

    /* velocity magnitude + duration under which we call it "landed" */
    float landed_velocity_mps;
    uint32_t landed_duration_ms;

    fc_pyro_fire_fn pyro_fire;
    void *pyro_ctx;
} fc_flight_state_config_t;

typedef struct {
    fc_flight_state_config_t cfg;
    fc_flight_phase_t phase;

    int boost_debounce_count;
    int apogee_debounce_count;

    int64_t phase_entered_us;
    int64_t boost_start_us;
    int64_t landed_candidate_since_us;

    uint8_t pyro_fired[FC_PYRO_CHANNEL_COUNT];
} fc_flight_state_t;

void fc_flight_state_config_default(fc_flight_state_config_t *cfg);
void fc_flight_state_init(fc_flight_state_t *s, const fc_flight_state_config_t *cfg);

/* Explicit arming (e.g. from a ground command / arm switch read by the
 * firmware). No-op unless currently in FC_PHASE_PAD. */
void fc_flight_state_arm(fc_flight_state_t *s, int64_t now_us);

/* Advance the state machine with a new fused state estimate. May call
 * cfg.pyro_fire synchronously. */
void fc_flight_state_update(fc_flight_state_t *s, const fc_state_estimate_t *state);

#ifdef __cplusplus
}
#endif
