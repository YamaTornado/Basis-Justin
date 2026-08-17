#pragma once

#include "fc/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Madgwick AHRS filter: fuses gyro + accel (+ optional mag) into an
 * orientation quaternion. Runs at the IMU sample rate. */

typedef struct {
    float q[4];       /* current orientation estimate, world<-body [w,x,y,z] */
    float beta;        /* filter gain: higher = trust accel/mag more, lower = trust gyro more */
    int have_mag;       /* 0 -> use IMU-only (gyro+accel) update, no mag */
} fc_attitude_filter_t;

/* beta: 0.03-0.1 is a reasonable starting range for a hand-launched rocket
 * (fast dynamics -> keep it on the lower/slower-converging side to avoid
 * accel-induced errors during boost). */
void fc_attitude_filter_init(fc_attitude_filter_t *f, float beta, int have_mag);

/* Advance the filter by one IMU sample. mag_ut may be NULL if no
 * magnetometer reading is available for this step (falls back to IMU-only
 * update for that step). dt_s must be > 0. */
void fc_attitude_filter_update(fc_attitude_filter_t *f, const float gyro_dps[3],
                                const float accel_g[3], const float mag_ut[3], float dt_s);

/* Rotate a body-frame vector into the world frame using the current
 * orientation estimate. */
void fc_attitude_filter_rotate_to_world(const fc_attitude_filter_t *f, const float body[3],
                                         float world_out[3]);

#ifdef __cplusplus
}
#endif
