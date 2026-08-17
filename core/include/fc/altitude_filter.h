#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Linear 3-state Kalman filter: state x = [altitude_m, velocity_mps, accel_bias_mps2].
 * Predict step is driven by world-frame vertical acceleration (from the
 * attitude filter, gravity already removed). Correct step uses barometric
 * altitude (and optionally, heavily de-weighted, GPS altitude).
 *
 * This is the classic "baro rocket altimeter" filter design (as used in
 * many hobby dual-deploy altimeters): simple, well understood, and cheap
 * enough to run at 100 Hz on an ESP32 with headroom to spare. */

typedef struct {
    float x[3];     /* [altitude_m, velocity_mps, accel_bias_mps2] */
    float p[3][3];  /* state covariance */

    float process_noise_accel;  /* variance of unmodeled accel (m/s^2)^2 */
    float process_noise_bias;   /* variance of bias random walk */
    float measurement_noise_baro; /* variance of baro altitude measurement (m^2) */
    float measurement_noise_gps;  /* variance of gps altitude measurement (m^2) */
} fc_altitude_filter_t;

void fc_altitude_filter_init(fc_altitude_filter_t *f, float initial_altitude_m);

/* Predict step: dt_s since last call, vertical_accel_mps2 = world-frame
 * vertical acceleration with gravity removed (0 at rest). */
void fc_altitude_filter_predict(fc_altitude_filter_t *f, float vertical_accel_mps2, float dt_s);

/* Correct with a barometric altitude measurement (relative to launch pad,
 * i.e. already zeroed against the pad pressure). */
void fc_altitude_filter_update_baro(fc_altitude_filter_t *f, float baro_altitude_m);

/* Correct with a GPS altitude measurement (also pad-relative). Call only
 * when a fresh GPS fix is available; noise is set high by default since
 * GPS altitude accuracy is coarse relative to baro. */
void fc_altitude_filter_update_gps(fc_altitude_filter_t *f, float gps_altitude_m);

#ifdef __cplusplus
}
#endif
