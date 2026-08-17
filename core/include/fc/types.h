#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Raw sensor samples, as produced by the driver layer in firmware/ (or by
 * the simulator/replay tool in sim/). Units are SI unless noted otherwise. */

typedef struct {
    int64_t timestamp_us;
    float accel_g[3];  /* body frame, g (1g = 9.80665 m/s^2) */
    float gyro_dps[3]; /* body frame, degrees/s */
    float mag_ut[3];   /* body frame, micro-tesla */
    float temp_c;
} fc_imu_sample_t;

typedef struct {
    int64_t timestamp_us;
    float pressure_pa;
    float temperature_c;
} fc_baro_sample_t;

typedef struct {
    int64_t timestamp_us;
    double latitude_deg;
    double longitude_deg;
    float altitude_m; /* MSL, from GPS fix */
    float speed_mps;
    uint8_t fix_type; /* 0 = no fix, 1 = 2D, 2 = 3D */
    uint8_t num_sats;
} fc_gps_sample_t;

/* Fused state estimate, output of the estimator (attitude filter + altitude
 * filter). This is the interface the rest of the system (flight state
 * machine, logger, telemetry) consumes -- kept stable even if the filters
 * behind it change. */
typedef struct {
    int64_t timestamp_us;
    float quat[4];              /* orientation, world<-body, [w, x, y, z] */
    float altitude_m;           /* fused, relative to launch pad (AGL) */
    float velocity_mps;         /* vertical, +up */
    float vertical_accel_mps2;  /* world-frame vertical accel, gravity removed */
    float accel_bias_mps2;      /* estimated vertical accelerometer bias */
} fc_state_estimate_t;

typedef enum {
    FC_PHASE_PAD = 0,
    FC_PHASE_ARMED,
    FC_PHASE_BOOST,
    FC_PHASE_COAST,
    FC_PHASE_APOGEE,
    FC_PHASE_DROGUE_DESCENT,
    FC_PHASE_MAIN_DESCENT,
    FC_PHASE_LANDED,
    FC_PHASE_ABORT,
    FC_PHASE_COUNT
} fc_flight_phase_t;

const char *fc_flight_phase_name(fc_flight_phase_t phase);

#ifdef __cplusplus
}
#endif
