#pragma once

/* Synthetic flight profile: a self-consistent piecewise-constant-
 * acceleration model of a single-deploy... actually dual-deploy (drogue +
 * main) rocket flight. "Self-consistent" means velocity/altitude are exact
 * closed-form integrals of the acceleration, so feeding profile_accel(t)
 * into fc_altitude_filter_predict() and comparing against profile_altitude(t)
 * is a meaningful test of the filter, not just noise-fitting.
 *
 * Vertical-only: no rotation is simulated (rocket assumed to fly straight
 * up), so the attitude filter isn't exercised here -- see main.c for how
 * sim mode bypasses it. Use replay mode (real recorded IMU/mag data) to
 * exercise the attitude filter. */

typedef struct {
    double t_start_s, v_start_mps, alt_start_m, accel_mps2, duration_s;
} profile_segment_t;

#define PROFILE_MAX_SEGMENTS 8

typedef struct {
    profile_segment_t segments[PROFILE_MAX_SEGMENTS];
    int segment_count;
    double total_duration_s;
} flight_profile_t;

/* Builds the default demo profile: ~2s boost to ~180 m/s^2, ballistic
 * coast to apogee, drogue deploy + descent, main deploy at 200m AGL,
 * landing. */
void profile_build_default(flight_profile_t *p);

/* True vertical acceleration at time t (m/s^2, same convention as
 * fc_state_estimate_t.vertical_accel_mps2: 0 at rest/terminal velocity,
 * positive while thrusting/decelerating upward). */
double profile_accel(const flight_profile_t *p, double t_s);

/* True altitude (m AGL) and velocity (m/s, +up) at time t, clamped to 0
 * altitude after landing. */
double profile_altitude(const flight_profile_t *p, double t_s);
double profile_velocity(const flight_profile_t *p, double t_s);
