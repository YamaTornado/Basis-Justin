#pragma once

#include <stddef.h>
#include <stdint.h>

#include "fc/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Binary log record format written to the flight_log flash partition (and
 * read back by both the ground tool and sim/'s replay mode). Every record
 * is a fc_framing frame: [type][len][payload][crc8]. Payloads are packed
 * structs below -- keep them fixed-size and append-only (add new record
 * types rather than changing existing payload layouts) so old logs stay
 * decodable. */

typedef enum {
    FC_LOG_REC_IMU = 1,
    FC_LOG_REC_BARO = 2,
    FC_LOG_REC_GPS = 3,
    FC_LOG_REC_STATE = 4,
    FC_LOG_REC_PHASE_CHANGE = 5,
} fc_log_record_type_t;

#pragma pack(push, 1)
typedef struct {
    int64_t timestamp_us;
    int8_t old_phase;
    int8_t new_phase;
} fc_log_phase_change_t;
#pragma pack(pop)

/* Maximum payload size any record type can produce; sizes the caller's
 * scratch buffer (2 + payload + 1). */
#define FC_LOG_MAX_PAYLOAD 64
#define FC_LOG_MAX_FRAME (FC_LOG_MAX_PAYLOAD + 3)

/* Each fc_log_encode_* writes a full frame into out (out_cap >= FC_LOG_MAX_FRAME)
 * and returns the number of bytes written, or 0 on error. */
size_t fc_log_encode_imu(const fc_imu_sample_t *s, uint8_t *out, size_t out_cap);
size_t fc_log_encode_baro(const fc_baro_sample_t *s, uint8_t *out, size_t out_cap);
size_t fc_log_encode_gps(const fc_gps_sample_t *s, uint8_t *out, size_t out_cap);
size_t fc_log_encode_state(const fc_state_estimate_t *s, uint8_t *out, size_t out_cap);
size_t fc_log_encode_phase_change(int64_t timestamp_us, fc_flight_phase_t old_phase,
                                   fc_flight_phase_t new_phase, uint8_t *out, size_t out_cap);

/* Decoded union for fc_log_decode_next(). */
typedef struct {
    fc_log_record_type_t type;
    union {
        fc_imu_sample_t imu;
        fc_baro_sample_t baro;
        fc_gps_sample_t gps;
        fc_state_estimate_t state;
        fc_log_phase_change_t phase_change;
    } as;
} fc_log_record_t;

/* Scans buf for the next valid, decodable record. Returns bytes consumed
 * (> 0, advance buf by this much and call again) on success, 0 if no
 * complete/valid frame is found in buf (need more data). On a CRC mismatch
 * or unknown type, resyncs by skipping one byte at a time internally --
 * callers just keep calling until 0 is returned. */
size_t fc_log_decode_next(const uint8_t *buf, size_t len, fc_log_record_t *out);

#ifdef __cplusplus
}
#endif
