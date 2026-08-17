#include "fc/altitude_filter.h"

#include <string.h>

void fc_altitude_filter_init(fc_altitude_filter_t *f, float initial_altitude_m) {
    memset(f, 0, sizeof(*f));
    f->x[0] = initial_altitude_m;
    f->x[1] = 0.0f;
    f->x[2] = 0.0f;

    /* Start with generous uncertainty; the first few baro updates will pull
     * this in quickly. */
    f->p[0][0] = 4.0f;   /* +/- 2 m */
    f->p[1][1] = 4.0f;   /* +/- 2 m/s */
    f->p[2][2] = 0.25f;  /* +/- 0.5 m/s^2 bias */

    f->process_noise_accel = 0.5f;    /* (m/s^2)^2 */
    f->process_noise_bias = 0.0005f;  /* (m/s^2)^2 per second */
    f->measurement_noise_baro = 0.36f; /* ~0.6 m std dev */
    f->measurement_noise_gps = 25.0f;  /* ~5 m std dev */
}

void fc_altitude_filter_predict(fc_altitude_filter_t *f, float vertical_accel_mps2, float dt_s) {
    if (dt_s <= 0.0f) {
        return;
    }

    float alt = f->x[0], vel = f->x[1], bias = f->x[2];
    float dt = dt_s, dt2 = dt * dt;

    /* State propagation: alt/vel driven by (measured accel - bias), bias is
     * a random walk. */
    float new_alt = alt + vel * dt + 0.5f * dt2 * (vertical_accel_mps2 - bias);
    float new_vel = vel + dt * (vertical_accel_mps2 - bias);
    float new_bias = bias;

    /* Jacobian F of the state transition w.r.t. x = [alt, vel, bias] */
    float F[3][3] = {
        {1.0f, dt, -0.5f * dt2},
        {0.0f, 1.0f, -dt},
        {0.0f, 0.0f, 1.0f},
    };

    /* P = F P F^T + Q */
    float FP[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            FP[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                FP[i][j] += F[i][k] * f->p[k][j];
            }
        }
    }
    float newP[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            newP[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                newP[i][j] += FP[i][k] * F[j][k]; /* F^T[k][j] = F[j][k] */
            }
        }
    }

    /* Process noise: unmodeled accel noise injected through B=[0.5dt^2, dt, 0],
     * plus independent bias random-walk noise. */
    float B[3] = {0.5f * dt2, dt, 0.0f};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            newP[i][j] += f->process_noise_accel * B[i] * B[j];
        }
    }
    newP[2][2] += f->process_noise_bias * dt;

    f->x[0] = new_alt;
    f->x[1] = new_vel;
    f->x[2] = new_bias;
    memcpy(f->p, newP, sizeof(newP));
}

static void altitude_filter_scalar_update(fc_altitude_filter_t *f, float measurement,
                                           float measurement_noise) {
    /* H = [1, 0, 0] -- direct altitude measurement */
    float y = measurement - f->x[0];
    float S = f->p[0][0] + measurement_noise;
    if (S <= 1e-9f) {
        return;
    }
    float K[3] = {f->p[0][0] / S, f->p[1][0] / S, f->p[2][0] / S};

    f->x[0] += K[0] * y;
    f->x[1] += K[1] * y;
    f->x[2] += K[2] * y;

    /* P = (I - K H) P */
    float newP[3][3];
    for (int j = 0; j < 3; j++) {
        newP[0][j] = f->p[0][j] - K[0] * f->p[0][j];
        newP[1][j] = f->p[1][j] - K[1] * f->p[0][j];
        newP[2][j] = f->p[2][j] - K[2] * f->p[0][j];
    }
    memcpy(f->p, newP, sizeof(newP));
}

void fc_altitude_filter_update_baro(fc_altitude_filter_t *f, float baro_altitude_m) {
    altitude_filter_scalar_update(f, baro_altitude_m, f->measurement_noise_baro);
}

void fc_altitude_filter_update_gps(fc_altitude_filter_t *f, float gps_altitude_m) {
    altitude_filter_scalar_update(f, gps_altitude_m, f->measurement_noise_gps);
}
