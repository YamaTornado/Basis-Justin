#include "fc/log_format.h"

#include <string.h>

#include "fc/framing.h"

/* Manual field-by-field (de)serialization instead of reinterpreting whole
 * structs as bytes: avoids relying on identical struct padding/alignment
 * between the ESP32 target and host tooling (sim/, future log_dump). All
 * multi-byte fields are written little-endian, which matches both Xtensa/
 * RISC-V ESP32 and typical host CPUs. */

static void put_bytes(uint8_t *buf, size_t *off, const void *src, size_t n) {
    memcpy(buf + *off, src, n);
    *off += n;
}

static void get_bytes(const uint8_t *buf, size_t *off, void *dst, size_t n) {
    memcpy(dst, buf + *off, n);
    *off += n;
}

size_t fc_log_encode_imu(const fc_imu_sample_t *s, uint8_t *out, size_t out_cap) {
    uint8_t payload[48];
    size_t off = 0;
    put_bytes(payload, &off, &s->timestamp_us, 8);
    put_bytes(payload, &off, s->accel_g, 12);
    put_bytes(payload, &off, s->gyro_dps, 12);
    put_bytes(payload, &off, s->mag_ut, 12);
    put_bytes(payload, &off, &s->temp_c, 4);
    return fc_frame_encode(FC_LOG_REC_IMU, payload, (uint8_t)off, out, out_cap);
}

size_t fc_log_encode_baro(const fc_baro_sample_t *s, uint8_t *out, size_t out_cap) {
    uint8_t payload[16];
    size_t off = 0;
    put_bytes(payload, &off, &s->timestamp_us, 8);
    put_bytes(payload, &off, &s->pressure_pa, 4);
    put_bytes(payload, &off, &s->temperature_c, 4);
    return fc_frame_encode(FC_LOG_REC_BARO, payload, (uint8_t)off, out, out_cap);
}

size_t fc_log_encode_gps(const fc_gps_sample_t *s, uint8_t *out, size_t out_cap) {
    uint8_t payload[34];
    size_t off = 0;
    put_bytes(payload, &off, &s->timestamp_us, 8);
    put_bytes(payload, &off, &s->latitude_deg, 8);
    put_bytes(payload, &off, &s->longitude_deg, 8);
    put_bytes(payload, &off, &s->altitude_m, 4);
    put_bytes(payload, &off, &s->speed_mps, 4);
    put_bytes(payload, &off, &s->fix_type, 1);
    put_bytes(payload, &off, &s->num_sats, 1);
    return fc_frame_encode(FC_LOG_REC_GPS, payload, (uint8_t)off, out, out_cap);
}

size_t fc_log_encode_state(const fc_state_estimate_t *s, uint8_t *out, size_t out_cap) {
    uint8_t payload[40];
    size_t off = 0;
    put_bytes(payload, &off, &s->timestamp_us, 8);
    put_bytes(payload, &off, s->quat, 16);
    put_bytes(payload, &off, &s->altitude_m, 4);
    put_bytes(payload, &off, &s->velocity_mps, 4);
    put_bytes(payload, &off, &s->vertical_accel_mps2, 4);
    put_bytes(payload, &off, &s->accel_bias_mps2, 4);
    return fc_frame_encode(FC_LOG_REC_STATE, payload, (uint8_t)off, out, out_cap);
}

size_t fc_log_encode_phase_change(int64_t timestamp_us, fc_flight_phase_t old_phase,
                                   fc_flight_phase_t new_phase, uint8_t *out, size_t out_cap) {
    uint8_t payload[10];
    size_t off = 0;
    int8_t old_p = (int8_t)old_phase, new_p = (int8_t)new_phase;
    put_bytes(payload, &off, &timestamp_us, 8);
    put_bytes(payload, &off, &old_p, 1);
    put_bytes(payload, &off, &new_p, 1);
    return fc_frame_encode(FC_LOG_REC_PHASE_CHANGE, payload, (uint8_t)off, out, out_cap);
}

static int decode_payload(fc_log_record_type_t type, const uint8_t *payload, uint8_t len,
                           fc_log_record_t *out) {
    size_t off = 0;
    switch (type) {
        case FC_LOG_REC_IMU:
            if (len != 48) return 0;
            get_bytes(payload, &off, &out->as.imu.timestamp_us, 8);
            get_bytes(payload, &off, out->as.imu.accel_g, 12);
            get_bytes(payload, &off, out->as.imu.gyro_dps, 12);
            get_bytes(payload, &off, out->as.imu.mag_ut, 12);
            get_bytes(payload, &off, &out->as.imu.temp_c, 4);
            return 1;
        case FC_LOG_REC_BARO:
            if (len != 16) return 0;
            get_bytes(payload, &off, &out->as.baro.timestamp_us, 8);
            get_bytes(payload, &off, &out->as.baro.pressure_pa, 4);
            get_bytes(payload, &off, &out->as.baro.temperature_c, 4);
            return 1;
        case FC_LOG_REC_GPS:
            if (len != 34) return 0;
            get_bytes(payload, &off, &out->as.gps.timestamp_us, 8);
            get_bytes(payload, &off, &out->as.gps.latitude_deg, 8);
            get_bytes(payload, &off, &out->as.gps.longitude_deg, 8);
            get_bytes(payload, &off, &out->as.gps.altitude_m, 4);
            get_bytes(payload, &off, &out->as.gps.speed_mps, 4);
            get_bytes(payload, &off, &out->as.gps.fix_type, 1);
            get_bytes(payload, &off, &out->as.gps.num_sats, 1);
            return 1;
        case FC_LOG_REC_STATE:
            if (len != 40) return 0;
            get_bytes(payload, &off, &out->as.state.timestamp_us, 8);
            get_bytes(payload, &off, out->as.state.quat, 16);
            get_bytes(payload, &off, &out->as.state.altitude_m, 4);
            get_bytes(payload, &off, &out->as.state.velocity_mps, 4);
            get_bytes(payload, &off, &out->as.state.vertical_accel_mps2, 4);
            get_bytes(payload, &off, &out->as.state.accel_bias_mps2, 4);
            return 1;
        case FC_LOG_REC_PHASE_CHANGE:
            if (len != 10) return 0;
            get_bytes(payload, &off, &out->as.phase_change.timestamp_us, 8);
            get_bytes(payload, &off, &out->as.phase_change.old_phase, 1);
            get_bytes(payload, &off, &out->as.phase_change.new_phase, 1);
            return 1;
        default:
            return 0;
    }
}

size_t fc_log_decode_next(const uint8_t *buf, size_t len, fc_log_record_t *out) {
    size_t skip = 0;
    while (skip < len) {
        fc_frame_t frame;
        size_t consumed = fc_frame_decode(buf + skip, len - skip, &frame);
        if (consumed == 0) {
            /* Either not enough bytes left for any frame, or the frame at
             * this offset is corrupt -- try resyncing one byte later. */
            if (len - skip < 3) {
                return 0; /* need more data */
            }
            skip++;
            continue;
        }
        if (decode_payload((fc_log_record_type_t)frame.type, frame.payload, frame.payload_len,
                            out)) {
            out->type = (fc_log_record_type_t)frame.type;
            return skip + consumed;
        }
        /* Valid frame (CRC ok) but unknown/malformed type -- skip past it
         * and keep scanning. */
        skip += consumed;
    }
    return 0;
}
