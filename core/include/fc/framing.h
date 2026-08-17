#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Shared byte-level framing used by both log_format.h (flash records) and
 * telemetry.h (LoRa packets): [type:1][len:1][payload:len][crc8:1].
 * Kept tiny and dependency-free so it can be re-decoded by a host tool with
 * nothing but this header + a copy of the struct layouts. */

uint8_t fc_crc8(const uint8_t *data, size_t len);

/* Encodes into out (must have room for 2 + payload_len + 1 bytes). Returns
 * the number of bytes written, or 0 if out_cap is too small. */
size_t fc_frame_encode(uint8_t type, const uint8_t *payload, uint8_t payload_len, uint8_t *out,
                        size_t out_cap);

typedef struct {
    uint8_t type;
    const uint8_t *payload;
    uint8_t payload_len;
} fc_frame_t;

/* Attempts to decode one frame starting at buf[0]. Returns the number of
 * bytes consumed (> 0) on success, 0 if buf does not contain a complete
 * valid frame yet (need more data / resync), writing the decoded frame into
 * *out on success. */
size_t fc_frame_decode(const uint8_t *buf, size_t len, fc_frame_t *out);

#ifdef __cplusplus
}
#endif
