#pragma once

#include <stddef.h>
#include <stdint.h>

#include "fc/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Compact telemetry packet for the LoRa downlink. Deliberately small (LoRa
 * airtime is precious): one "status" packet type covers everything ground
 * needs live. Full-resolution data lives in the flash log, not on the air.
 * Uses the same fc_framing wire format as log_format.h, with an added
 * sequence number so ground can detect drops (LoRa link is best-effort). */

typedef enum {
    FC_TLM_STATUS = 1,
} fc_telemetry_type_t;

typedef struct {
    uint16_t seq;
    int64_t timestamp_us;
    int8_t phase; /* fc_flight_phase_t */
    float altitude_m;
    float velocity_mps;
    double latitude_deg;
    double longitude_deg;
    uint8_t gps_fix_type;
    uint8_t num_sats;
    uint8_t battery_pct; /* 0-100, 0xFF = unknown */
} fc_telemetry_status_t;

#define FC_TLM_MAX_FRAME 48

/* Writes a full frame into out (out_cap >= FC_TLM_MAX_FRAME). Returns bytes
 * written, or 0 on error. seq is caller-managed (increment per packet). */
size_t fc_telemetry_encode_status(const fc_telemetry_status_t *s, uint8_t *out, size_t out_cap);

/* Decodes one status packet. Returns 1 on success + bytes consumed written
 * to *consumed, 0 if buf has no complete/valid frame yet. */
int fc_telemetry_decode_status(const uint8_t *buf, size_t len, fc_telemetry_status_t *out,
                                size_t *consumed);

#ifdef __cplusplus
}
#endif
