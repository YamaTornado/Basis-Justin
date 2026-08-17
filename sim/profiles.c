#include "profiles.h"

#include <math.h>
#include <stddef.h>

#define GRAVITY 9.80665

static void add_segment(flight_profile_t *p, double accel, double duration) {
    profile_segment_t *seg = &p->segments[p->segment_count];
    const profile_segment_t *prev =
        p->segment_count > 0 ? &p->segments[p->segment_count - 1] : NULL;

    seg->t_start_s = prev ? prev->t_start_s + prev->duration_s : 0.0;
    seg->v_start_mps = prev ? prev->v_start_mps + prev->accel_mps2 * prev->duration_s : 0.0;
    seg->alt_start_m = prev ? prev->alt_start_m +
                                   prev->v_start_mps * prev->duration_s +
                                   0.5 * prev->accel_mps2 * prev->duration_s * prev->duration_s
                             : 0.0;
    seg->accel_mps2 = accel;
    seg->duration_s = duration;

    p->segment_count++;
    p->total_duration_s = seg->t_start_s + seg->duration_s;
}

void profile_build_default(flight_profile_t *p) {
    p->segment_count = 0;

    /* ~5g net upward accel for 1.5s -- a plausible mid-power hobby rocket
     * boost (burnout ~75 m/s, apogee ~340m). The earlier +180 m/s^2/2s draft
     * of this profile reached mach speeds and a multi-km apogee, which
     * masked real apogee detection behind flight_state's 30s backup timer
     * (see fc_flight_state_config_t.max_coast_time_ms) -- caught by running
     * `fc_sim sim` and noticing drogue fired at exactly t=30s. */
    const double boost_accel = 50.0;
    const double boost_duration = 1.5;
    add_segment(p, boost_accel, boost_duration);

    /* Coast: ballistic, unpowered -- accelerometer reads 0 (free fall),
     * so vertical_accel_mps2 = -g in this convention (see fc/types.h). */
    double v_at_burnout = boost_accel * boost_duration;
    double coast_duration = v_at_burnout / GRAVITY; /* until v=0 (apogee) */
    add_segment(p, -GRAVITY, coast_duration);

    /* Drogue deploy: rapid deceleration from 0 to drogue terminal velocity. */
    const double drogue_terminal_mps = -20.0;
    const double drogue_open_s = 1.0;
    add_segment(p, drogue_terminal_mps / drogue_open_s, drogue_open_s);

    /* Drogue terminal descent: constant velocity until main deploy altitude. */
    double alt_after_drogue_open =
        p->segments[2].alt_start_m + p->segments[2].v_start_mps * drogue_open_s +
        0.5 * p->segments[2].accel_mps2 * drogue_open_s * drogue_open_s;
    const double main_deploy_alt_m = 200.0;
    double drogue_terminal_duration =
        (alt_after_drogue_open - main_deploy_alt_m) / (-drogue_terminal_mps);
    add_segment(p, 0.0, drogue_terminal_duration);

    /* Main deploy: rapid deceleration from drogue rate to main terminal velocity. */
    const double main_terminal_mps = -5.0;
    const double main_open_s = 1.0;
    add_segment(p, (main_terminal_mps - drogue_terminal_mps) / main_open_s, main_open_s);

    /* Main terminal descent: constant velocity until touchdown (alt=0). */
    double alt_after_main_open =
        p->segments[4].alt_start_m + p->segments[4].v_start_mps * main_open_s +
        0.5 * p->segments[4].accel_mps2 * main_open_s * main_open_s;
    double main_terminal_duration = alt_after_main_open / (-main_terminal_mps);
    add_segment(p, 0.0, main_terminal_duration);

    /* Landed: stay put. */
    add_segment(p, 0.0, 30.0);
}

static const profile_segment_t *segment_at(const flight_profile_t *p, double t_s) {
    if (t_s >= p->total_duration_s) {
        return &p->segments[p->segment_count - 1];
    }
    for (int i = 0; i < p->segment_count; i++) {
        const profile_segment_t *seg = &p->segments[i];
        if (t_s < seg->t_start_s + seg->duration_s) {
            return seg;
        }
    }
    return &p->segments[p->segment_count - 1];
}

double profile_accel(const flight_profile_t *p, double t_s) {
    return segment_at(p, t_s)->accel_mps2;
}

double profile_velocity(const flight_profile_t *p, double t_s) {
    const profile_segment_t *seg = segment_at(p, t_s);
    double local_t = t_s - seg->t_start_s;
    if (local_t > seg->duration_s) local_t = seg->duration_s; /* clamp past end (landed segment) */
    return seg->v_start_mps + seg->accel_mps2 * local_t;
}

double profile_altitude(const flight_profile_t *p, double t_s) {
    const profile_segment_t *seg = segment_at(p, t_s);
    double local_t = t_s - seg->t_start_s;
    if (local_t > seg->duration_s) local_t = seg->duration_s;
    double alt = seg->alt_start_m + seg->v_start_mps * local_t +
                 0.5 * seg->accel_mps2 * local_t * local_t;
    return alt < 0.0 ? 0.0 : alt;
}
