/* Host-side simulator + replay tool. Links fc_core directly (see
 * ../core/CMakeLists.txt) -- no ESP-IDF, no hardware. Two modes:
 *
 *   sim/build/fc_sim sim              synthetic flight profile (profiles.c)
 *   sim/build/fc_sim replay <logfile> real flight_log dump from the device
 *
 * Both drive the exact same fc_flight_state_t / filter code the firmware
 * uses, so this is a real test of the deployment logic, not just a
 * standalone toy. See docs/MISSION_GOALS.md, goal 6 "Verify". */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fc/altitude_filter.h"
#include "fc/attitude_filter.h"
#include "fc/flight_state.h"
#include "fc/log_format.h"
#include "profiles.h"

static int g_pyro_fire_count[FC_PYRO_CHANNEL_COUNT];

static void on_pyro_fire(fc_pyro_channel_t channel, void *ctx) {
    (void)ctx;
    g_pyro_fire_count[channel]++;
    fprintf(stderr, "*** PYRO FIRE: %s at count %d ***\n",
            channel == FC_PYRO_DROGUE ? "DROGUE" : "MAIN", g_pyro_fire_count[channel]);
}

static double rand_noise(double amplitude) {
    return amplitude * (2.0 * ((double)rand() / (double)RAND_MAX) - 1.0);
}

static void print_csv_header(void) {
    printf("t_s,true_alt_m,true_vel_mps,est_alt_m,est_vel_mps,accel_bias,phase\n");
}

static void print_csv_row(double t, double true_alt, double true_vel,
                           const fc_altitude_filter_t *alt, fc_flight_phase_t phase) {
    printf("%.3f,%.2f,%.2f,%.2f,%.2f,%.3f,%s\n", t, true_alt, true_vel, alt->x[0], alt->x[1],
           alt->x[2], fc_flight_phase_name(phase));
}

static int run_sim(void) {
    flight_profile_t profile;
    profile_build_default(&profile);

    fc_altitude_filter_t alt_filter;
    fc_altitude_filter_init(&alt_filter, 0.0f);

    fc_flight_state_config_t cfg;
    fc_flight_state_config_default(&cfg);
    cfg.pyro_fire = on_pyro_fire;

    fc_flight_state_t state;
    fc_flight_state_init(&state, &cfg);

    print_csv_header();

    const double dt = 0.01; /* 100 Hz, matches sensor_task rate */
    const double baro_period = 0.02; /* 50 Hz baro updates */
    double next_baro_t = 0.0;
    double t = 0.0;

    fc_flight_state_arm(&state, 0);

    while (t <= profile.total_duration_s + 5.0) {
        double true_accel = profile_accel(&profile, t);
        double noisy_accel = true_accel + rand_noise(0.3); /* +/-0.3 m/s^2 accel noise */
        fc_altitude_filter_predict(&alt_filter, (float)noisy_accel, (float)dt);

        if (t >= next_baro_t) {
            double true_alt = profile_altitude(&profile, t);
            double noisy_alt = true_alt + rand_noise(0.5); /* +/-0.5 m baro noise */
            fc_altitude_filter_update_baro(&alt_filter, (float)noisy_alt);
            next_baro_t += baro_period;
        }

        fc_state_estimate_t s = {0};
        s.timestamp_us = (int64_t)(t * 1e6);
        s.quat[0] = 1.0f; /* vertical-only sim, no rotation modeled */
        s.altitude_m = alt_filter.x[0];
        s.velocity_mps = alt_filter.x[1];
        s.accel_bias_mps2 = alt_filter.x[2];
        s.vertical_accel_mps2 = (float)noisy_accel;

        fc_flight_phase_t prev_phase = state.phase;
        fc_flight_state_update(&state, &s);
        if (state.phase != prev_phase) {
            fprintf(stderr, "t=%.2fs  phase %s -> %s\n", t, fc_flight_phase_name(prev_phase),
                    fc_flight_phase_name(state.phase));
        }

        print_csv_row(t, profile_altitude(&profile, t), profile_velocity(&profile, t),
                      &alt_filter, state.phase);

        t += dt;
    }

    fprintf(stderr, "\nsummary: drogue fired %d time(s), main fired %d time(s)\n",
            g_pyro_fire_count[FC_PYRO_DROGUE], g_pyro_fire_count[FC_PYRO_MAIN]);
    return (g_pyro_fire_count[FC_PYRO_DROGUE] == 1 && g_pyro_fire_count[FC_PYRO_MAIN] == 1) ? 0
                                                                                              : 1;
}

static int run_replay(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "cannot open %s\n", path);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size <= 0) {
        fprintf(stderr, "empty or unreadable file: %s\n", path);
        fclose(f);
        return 1;
    }
    uint8_t *buf = (uint8_t *)malloc((size_t)size);
    if (!buf || fread(buf, 1, (size_t)size, f) != (size_t)size) {
        fprintf(stderr, "failed to read %s\n", path);
        fclose(f);
        free(buf);
        return 1;
    }
    fclose(f);

    fc_attitude_filter_t attitude;
    fc_attitude_filter_init(&attitude, 0.06f, /*have_mag=*/true);
    fc_altitude_filter_t alt_filter;
    fc_altitude_filter_init(&alt_filter, 0.0f);

    fc_flight_state_config_t cfg;
    fc_flight_state_config_default(&cfg);
    cfg.pyro_fire = on_pyro_fire;
    fc_flight_state_t state;
    fc_flight_state_init(&state, &cfg);
    fc_flight_state_arm(&state, 0);

    print_csv_header();

    float reference_pressure_pa = 0.0f;
    float pressure_accum = 0.0f;
    int pressure_count = 0;
    bool have_reference = false;
    int64_t last_imu_us = 0;
    bool have_last_imu = false;
    size_t record_count = 0;

    size_t offset = 0;
    while (offset < (size_t)size) {
        fc_log_record_t rec;
        size_t consumed = fc_log_decode_next(buf + offset, (size_t)size - offset, &rec);
        if (consumed == 0) {
            break; /* no more complete/valid frames */
        }
        offset += consumed;
        record_count++;

        if (rec.type == FC_LOG_REC_BARO) {
            if (!have_reference) {
                pressure_accum += rec.as.baro.pressure_pa;
                pressure_count++;
                if (pressure_count >= 50) {
                    reference_pressure_pa = pressure_accum / (float)pressure_count;
                    have_reference = true;
                }
                continue;
            }
            double baro_alt =
                44330.0 * (1.0 - pow(rec.as.baro.pressure_pa / reference_pressure_pa, 0.1903));
            fc_altitude_filter_update_baro(&alt_filter, (float)baro_alt);
        } else if (rec.type == FC_LOG_REC_IMU && have_reference) {
            int64_t now_us = rec.as.imu.timestamp_us;
            float dt_s = have_last_imu ? (float)(now_us - last_imu_us) / 1e6f : 0.01f;
            if (dt_s <= 0.0f || dt_s > 0.5f) dt_s = 0.01f;
            last_imu_us = now_us;
            have_last_imu = true;

            fc_attitude_filter_update(&attitude, rec.as.imu.gyro_dps, rec.as.imu.accel_g,
                                       rec.as.imu.mag_ut, dt_s);
            float world_g[3];
            fc_attitude_filter_rotate_to_world(&attitude, rec.as.imu.accel_g, world_g);
            float vertical_accel = (world_g[2] - 1.0f) * 9.80665f;
            fc_altitude_filter_predict(&alt_filter, vertical_accel, dt_s);

            fc_state_estimate_t s = {0};
            s.timestamp_us = now_us;
            memcpy(s.quat, attitude.q, sizeof(s.quat));
            s.altitude_m = alt_filter.x[0];
            s.velocity_mps = alt_filter.x[1];
            s.accel_bias_mps2 = alt_filter.x[2];
            s.vertical_accel_mps2 = vertical_accel;

            fc_flight_phase_t prev_phase = state.phase;
            fc_flight_state_update(&state, &s);
            if (state.phase != prev_phase) {
                fprintf(stderr, "t=%.3fus  phase %s -> %s\n", (double)now_us,
                        fc_flight_phase_name(prev_phase), fc_flight_phase_name(state.phase));
            }
            print_csv_row((double)now_us / 1e6, 0.0, 0.0, &alt_filter, state.phase);
        }
    }

    fprintf(stderr, "\nreplayed %zu records from %s\n", record_count, path);
    fprintf(stderr, "summary: drogue fired %d time(s), main fired %d time(s)\n",
            g_pyro_fire_count[FC_PYRO_DROGUE], g_pyro_fire_count[FC_PYRO_MAIN]);

    free(buf);
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s sim | replay <logfile>\n", argv[0]);
        return 2;
    }
    if (strcmp(argv[1], "sim") == 0) {
        return run_sim();
    }
    if (strcmp(argv[1], "replay") == 0 && argc >= 3) {
        return run_replay(argv[2]);
    }
    fprintf(stderr, "usage: %s sim | replay <logfile>\n", argv[0]);
    return 2;
}
