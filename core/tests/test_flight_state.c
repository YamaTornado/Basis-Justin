#include "fc/flight_state.h"
#include "test_harness.h"

static int g_fire_count[FC_PYRO_CHANNEL_COUNT];
static int g_fire_order[FC_PYRO_CHANNEL_COUNT];
static int g_fire_seq = 0;

static void on_pyro_fire(fc_pyro_channel_t ch, void *ctx) {
    (void)ctx;
    g_fire_count[ch]++;
    g_fire_order[g_fire_seq++] = (int)ch;
}

static void update_state(fc_flight_state_t *s, int64_t t_us, float alt, float vel, float accel) {
    fc_state_estimate_t state = {0};
    state.timestamp_us = t_us;
    state.altitude_m = alt;
    state.velocity_mps = vel;
    state.vertical_accel_mps2 = accel;
    state.quat[0] = 1.0f;
    fc_flight_state_update(s, &state);
}

FC_TEST_MAIN_BEGIN()

fc_flight_state_config_t cfg;
fc_flight_state_config_default(&cfg);
cfg.boost_debounce_samples = 3;
cfg.apogee_debounce_samples = 3;
cfg.main_deploy_altitude_m = 100.0f;
cfg.landed_velocity_mps = 1.0f;
cfg.landed_duration_ms = 1000;
cfg.pyro_fire = on_pyro_fire;

fc_flight_state_t s;
fc_flight_state_init(&s, &cfg);
FC_CHECK(s.phase == FC_PHASE_PAD);

int64_t t = 0;
/* still on the pad: no transition without explicit arm */
update_state(&s, t, 0, 0, 0);
FC_CHECK(s.phase == FC_PHASE_PAD);

fc_flight_state_arm(&s, t);
FC_CHECK(s.phase == FC_PHASE_ARMED);

/* boost: sustained high accel */
for (int i = 0; i < 5; i++) {
    t += 10000; /* 10ms steps */
    update_state(&s, t, i * 0.5f, i * 5.0f, 30.0f);
}
FC_CHECK(s.phase == FC_PHASE_BOOST);

/* motor burnout: accel drops, velocity still positive -> COAST */
t += 10000;
update_state(&s, t, 50.0f, 60.0f, 0.0f);
FC_CHECK(s.phase == FC_PHASE_COAST);

/* coasting up, then velocity goes negative -> apogee -> drogue fires */
float vel = 60.0f;
float alt = 200.0f;
for (int i = 0; i < 6; i++) {
    t += 10000;
    vel -= 15.0f; /* crosses zero partway through */
    update_state(&s, t, alt, vel, -9.8f);
}
FC_CHECK(s.phase == FC_PHASE_DROGUE_DESCENT);
FC_CHECK(g_fire_count[FC_PYRO_DROGUE] == 1);
FC_CHECK(g_fire_count[FC_PYRO_MAIN] == 0);

/* descend under drogue past main deploy altitude */
alt = 150.0f;
for (int i = 0; i < 6; i++) {
    t += 100000; /* 100ms steps */
    alt -= 15.0f;
    update_state(&s, t, alt, -10.0f, 0.0f);
}
FC_CHECK(s.phase == FC_PHASE_MAIN_DESCENT);
FC_CHECK(g_fire_count[FC_PYRO_MAIN] == 1);

/* slow down under main and hold for landed_duration_ms -> LANDED */
for (int i = 0; i < 15; i++) {
    t += 100000; /* 100ms steps, 1.5s total > landed_duration_ms */
    update_state(&s, t, 0.0f, 0.2f, 0.0f);
}
FC_CHECK(s.phase == FC_PHASE_LANDED);

/* pyros must never fire twice, and drogue must fire before main */
FC_CHECK(g_fire_count[FC_PYRO_DROGUE] == 1);
FC_CHECK(g_fire_count[FC_PYRO_MAIN] == 1);
FC_CHECK(g_fire_seq == 2);
FC_CHECK(g_fire_order[0] == FC_PYRO_DROGUE);
FC_CHECK(g_fire_order[1] == FC_PYRO_MAIN);

FC_TEST_MAIN_END()
