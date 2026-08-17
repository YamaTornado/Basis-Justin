#include <string.h>

#include "fc/log_format.h"
#include "fc/telemetry.h"
#include "test_harness.h"

FC_TEST_MAIN_BEGIN()

/* IMU record round-trip */
{
    fc_imu_sample_t s = {.timestamp_us = 123456789,
                          .accel_g = {0.1f, -0.2f, 1.0f},
                          .gyro_dps = {1.0f, 2.0f, 3.0f},
                          .mag_ut = {10.0f, 20.0f, 30.0f},
                          .temp_c = 25.5f};
    uint8_t buf[FC_LOG_MAX_FRAME];
    size_t n = fc_log_encode_imu(&s, buf, sizeof(buf));
    FC_CHECK(n > 0);

    fc_log_record_t rec;
    size_t consumed = fc_log_decode_next(buf, n, &rec);
    FC_CHECK(consumed == n);
    FC_CHECK(rec.type == FC_LOG_REC_IMU);
    FC_CHECK(rec.as.imu.timestamp_us == s.timestamp_us);
    FC_CHECK_NEAR(rec.as.imu.accel_g[2], 1.0f, 1e-6);
    FC_CHECK_NEAR(rec.as.imu.temp_c, 25.5f, 1e-6);
}

/* State record round-trip */
{
    fc_state_estimate_t s = {0};
    s.timestamp_us = 42;
    s.quat[0] = 1.0f;
    s.altitude_m = 123.4f;
    s.velocity_mps = -5.5f;

    uint8_t buf[FC_LOG_MAX_FRAME];
    size_t n = fc_log_encode_state(&s, buf, sizeof(buf));

    fc_log_record_t rec;
    size_t consumed = fc_log_decode_next(buf, n, &rec);
    FC_CHECK(consumed == n);
    FC_CHECK(rec.type == FC_LOG_REC_STATE);
    FC_CHECK_NEAR(rec.as.state.altitude_m, 123.4f, 1e-4);
    FC_CHECK_NEAR(rec.as.state.velocity_mps, -5.5f, 1e-4);
}

/* Corrupted bytes before a valid record must be skipped (resync) */
{
    fc_baro_sample_t b = {.timestamp_us = 99, .pressure_pa = 101325.0f, .temperature_c = 20.0f};
    uint8_t buf[8 + FC_LOG_MAX_FRAME];
    memset(buf, 0xFF, 8);
    size_t n = fc_log_encode_baro(&b, buf + 8, sizeof(buf) - 8);

    fc_log_record_t rec;
    size_t consumed = fc_log_decode_next(buf, 8 + n, &rec);
    FC_CHECK(consumed == 8 + n);
    FC_CHECK(rec.type == FC_LOG_REC_BARO);
    FC_CHECK_NEAR(rec.as.baro.pressure_pa, 101325.0f, 1e-3);
}

/* Truncated buffer must report "need more data" (0), never a false decode */
{
    fc_gps_sample_t g = {.timestamp_us = 1, .latitude_deg = 52.5, .longitude_deg = 13.4,
                          .altitude_m = 34.0f, .speed_mps = 0.0f, .fix_type = 2, .num_sats = 9};
    uint8_t buf[FC_LOG_MAX_FRAME];
    size_t n = fc_log_encode_gps(&g, buf, sizeof(buf));

    fc_log_record_t rec;
    size_t consumed = fc_log_decode_next(buf, n - 1, &rec);
    FC_CHECK(consumed == 0);
}

/* Telemetry status round-trip */
{
    fc_telemetry_status_t t = {0};
    t.seq = 7;
    t.timestamp_us = 555;
    t.phase = (int8_t)FC_PHASE_MAIN_DESCENT;
    t.altitude_m = 88.8f;
    t.velocity_mps = -3.2f;
    t.latitude_deg = 52.520008;
    t.longitude_deg = 13.404954;
    t.gps_fix_type = 2;
    t.num_sats = 11;
    t.battery_pct = 87;

    uint8_t buf[FC_TLM_MAX_FRAME];
    size_t n = fc_telemetry_encode_status(&t, buf, sizeof(buf));
    FC_CHECK(n > 0);

    fc_telemetry_status_t out;
    size_t consumed = 0;
    int ok = fc_telemetry_decode_status(buf, n, &out, &consumed);
    FC_CHECK(ok == 1);
    FC_CHECK(consumed == n);
    FC_CHECK(out.seq == 7);
    FC_CHECK(out.phase == (int8_t)FC_PHASE_MAIN_DESCENT);
    FC_CHECK_NEAR(out.altitude_m, 88.8f, 1e-4);
    FC_CHECK_NEAR(out.latitude_deg, 52.520008, 1e-9);
    FC_CHECK(out.num_sats == 11);
    FC_CHECK(out.battery_pct == 87);
}

/* A single bit-flip must be caught by the CRC, not silently accepted */
{
    fc_baro_sample_t b = {.timestamp_us = 1, .pressure_pa = 100000.0f, .temperature_c = 15.0f};
    uint8_t buf[FC_LOG_MAX_FRAME];
    size_t n = fc_log_encode_baro(&b, buf, sizeof(buf));
    buf[2] ^= 0x01; /* corrupt one payload byte */

    fc_log_record_t rec;
    size_t consumed = fc_log_decode_next(buf, n, &rec);
    FC_CHECK(consumed == 0);
}

FC_TEST_MAIN_END()
