#include "fc/telemetry.h"

#include <string.h>

#include "fc/framing.h"

static void put_bytes(uint8_t *buf, size_t *off, const void *src, size_t n) {
    memcpy(buf + *off, src, n);
    *off += n;
}

static void get_bytes(const uint8_t *buf, size_t *off, void *dst, size_t n) {
    memcpy(dst, buf + *off, n);
    *off += n;
}

size_t fc_telemetry_encode_status(const fc_telemetry_status_t *s, uint8_t *out, size_t out_cap) {
    uint8_t payload[38];
    size_t off = 0;
    put_bytes(payload, &off, &s->seq, 2);
    put_bytes(payload, &off, &s->timestamp_us, 8);
    put_bytes(payload, &off, &s->phase, 1);
    put_bytes(payload, &off, &s->altitude_m, 4);
    put_bytes(payload, &off, &s->velocity_mps, 4);
    put_bytes(payload, &off, &s->latitude_deg, 8);
    put_bytes(payload, &off, &s->longitude_deg, 8);
    put_bytes(payload, &off, &s->gps_fix_type, 1);
    put_bytes(payload, &off, &s->num_sats, 1);
    put_bytes(payload, &off, &s->battery_pct, 1);
    return fc_frame_encode(FC_TLM_STATUS, payload, (uint8_t)off, out, out_cap);
}

int fc_telemetry_decode_status(const uint8_t *buf, size_t len, fc_telemetry_status_t *out,
                                size_t *consumed) {
    fc_frame_t frame;
    size_t n = fc_frame_decode(buf, len, &frame);
    if (n == 0 || frame.type != FC_TLM_STATUS || frame.payload_len != 38) {
        return 0;
    }
    size_t off = 0;
    get_bytes(frame.payload, &off, &out->seq, 2);
    get_bytes(frame.payload, &off, &out->timestamp_us, 8);
    get_bytes(frame.payload, &off, &out->phase, 1);
    get_bytes(frame.payload, &off, &out->altitude_m, 4);
    get_bytes(frame.payload, &off, &out->velocity_mps, 4);
    get_bytes(frame.payload, &off, &out->latitude_deg, 8);
    get_bytes(frame.payload, &off, &out->longitude_deg, 8);
    get_bytes(frame.payload, &off, &out->gps_fix_type, 1);
    get_bytes(frame.payload, &off, &out->num_sats, 1);
    get_bytes(frame.payload, &off, &out->battery_pct, 1);
    if (consumed) {
        *consumed = n;
    }
    return 1;
}
