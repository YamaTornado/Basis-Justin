#include "fc/flight_state.h"

#include <string.h>

const char *fc_flight_phase_name(fc_flight_phase_t phase) {
    switch (phase) {
        case FC_PHASE_PAD: return "PAD";
        case FC_PHASE_ARMED: return "ARMED";
        case FC_PHASE_BOOST: return "BOOST";
        case FC_PHASE_COAST: return "COAST";
        case FC_PHASE_APOGEE: return "APOGEE";
        case FC_PHASE_DROGUE_DESCENT: return "DROGUE_DESCENT";
        case FC_PHASE_MAIN_DESCENT: return "MAIN_DESCENT";
        case FC_PHASE_LANDED: return "LANDED";
        case FC_PHASE_ABORT: return "ABORT";
        default: return "UNKNOWN";
    }
}

void fc_flight_state_config_default(fc_flight_state_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->boost_accel_threshold_mps2 = 2.0f * 9.80665f; /* ~2g */
    cfg->boost_debounce_samples = 5;                    /* @100Hz -> 50ms */
    cfg->apogee_debounce_samples = 10;                   /* @100Hz -> 100ms */
    cfg->max_coast_time_ms = 30000;                      /* 30s backup */
    cfg->main_deploy_altitude_m = 200.0f;
    cfg->landed_velocity_mps = 1.0f;
    cfg->landed_duration_ms = 5000;
    cfg->pyro_fire = NULL;
    cfg->pyro_ctx = NULL;
}

void fc_flight_state_init(fc_flight_state_t *s, const fc_flight_state_config_t *cfg) {
    memset(s, 0, sizeof(*s));
    s->cfg = *cfg;
    s->phase = FC_PHASE_PAD;
}

static void enter_phase(fc_flight_state_t *s, fc_flight_phase_t phase, int64_t now_us) {
    s->phase = phase;
    s->phase_entered_us = now_us;
    s->boost_debounce_count = 0;
    s->apogee_debounce_count = 0;
}

static void fire_pyro(fc_flight_state_t *s, fc_pyro_channel_t channel) {
    if (s->pyro_fired[channel]) {
        return;
    }
    s->pyro_fired[channel] = 1;
    if (s->cfg.pyro_fire != NULL) {
        s->cfg.pyro_fire(channel, s->cfg.pyro_ctx);
    }
}

/* Shared by BOOST (once thrust drops) and COAST: detect apogee via a
 * debounced velocity sign flip, with a timeout-based backup deploy in case
 * the sign flip is never seen cleanly (sensor noise/failure). */
static void check_coast_and_apogee(fc_flight_state_t *s, float vel, int64_t now) {
    if (vel <= 0.0f) {
        s->apogee_debounce_count++;
        if (s->apogee_debounce_count >= s->cfg.apogee_debounce_samples) {
            fire_pyro(s, FC_PYRO_DROGUE);
            enter_phase(s, FC_PHASE_DROGUE_DESCENT, now);
            return;
        }
    } else {
        s->apogee_debounce_count = 0;
    }

    if (s->boost_start_us != 0 &&
        (uint32_t)((now - s->boost_start_us) / 1000) >= s->cfg.max_coast_time_ms) {
        /* backup: apogee not detected in time, deploy drogue anyway */
        fire_pyro(s, FC_PYRO_DROGUE);
        enter_phase(s, FC_PHASE_DROGUE_DESCENT, now);
    }
}

void fc_flight_state_arm(fc_flight_state_t *s, int64_t now_us) {
    if (s->phase == FC_PHASE_PAD) {
        enter_phase(s, FC_PHASE_ARMED, now_us);
    }
}

void fc_flight_state_update(fc_flight_state_t *s, const fc_state_estimate_t *state) {
    int64_t now = state->timestamp_us;
    float accel = state->vertical_accel_mps2;
    float vel = state->velocity_mps;
    float alt = state->altitude_m;

    switch (s->phase) {
        case FC_PHASE_PAD:
            /* waiting for explicit arm, see fc_flight_state_arm() */
            break;

        case FC_PHASE_ARMED:
            if (accel > s->cfg.boost_accel_threshold_mps2) {
                s->boost_debounce_count++;
                if (s->boost_debounce_count >= s->cfg.boost_debounce_samples) {
                    s->boost_start_us = now;
                    enter_phase(s, FC_PHASE_BOOST, now);
                }
            } else {
                s->boost_debounce_count = 0;
            }
            break;

        case FC_PHASE_BOOST:
            if (accel <= s->cfg.boost_accel_threshold_mps2) {
                enter_phase(s, FC_PHASE_COAST, now);
            }
            check_coast_and_apogee(s, vel, now);
            break;

        case FC_PHASE_COAST:
            check_coast_and_apogee(s, vel, now);
            break;

        case FC_PHASE_APOGEE:
            /* unused as a resting state: check_coast_and_apogee() jumps
             * straight from BOOST/COAST to DROGUE_DESCENT. Kept in the enum
             * for telemetry/logging to record the moment of apogee. */
            break;

        case FC_PHASE_DROGUE_DESCENT:
            if (alt <= s->cfg.main_deploy_altitude_m) {
                fire_pyro(s, FC_PYRO_MAIN);
                enter_phase(s, FC_PHASE_MAIN_DESCENT, now);
            }
            break;

        case FC_PHASE_MAIN_DESCENT: {
            float speed = vel < 0.0f ? -vel : vel;
            if (speed <= s->cfg.landed_velocity_mps) {
                if (s->landed_candidate_since_us == 0) {
                    s->landed_candidate_since_us = now;
                } else if ((uint32_t)((now - s->landed_candidate_since_us) / 1000) >=
                           s->cfg.landed_duration_ms) {
                    enter_phase(s, FC_PHASE_LANDED, now);
                }
            } else {
                s->landed_candidate_since_us = 0;
            }
            break;
        }

        case FC_PHASE_LANDED:
        case FC_PHASE_ABORT:
        case FC_PHASE_COUNT:
        default:
            break;
    }
}
