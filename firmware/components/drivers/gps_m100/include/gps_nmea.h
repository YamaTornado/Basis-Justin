#pragma once

#include <stdbool.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "fc/types.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HGLRC M100 MINI: u-blox-based GPS module, NMEA-0183 over UART (9600 baud
 * by default on most M100 MINI units -- verify against your module, some
 * ship pre-configured for a different rate). Parses GGA (fix/altitude/sats)
 * and RMC (lat/lon/speed) sentences; ignores everything else. */

typedef struct {
    uart_port_t uart_port;
    int tx_gpio;
    int rx_gpio;
    int baud_rate;
} gps_nmea_config_t;

typedef struct {
    uart_port_t uart_port;
    fc_gps_sample_t last; /* accumulated from the most recent GGA + RMC */
    char line_buf[96];
    size_t line_len;
} gps_nmea_t;

void gps_nmea_config_default(gps_nmea_config_t *cfg, uart_port_t port, int tx_gpio, int rx_gpio);

esp_err_t gps_nmea_init(gps_nmea_t *gps, const gps_nmea_config_t *cfg);

/* Non-blocking-ish: reads whatever is currently in the UART RX buffer
 * (bounded by timeout_ticks), parses any complete NMEA lines found, and
 * always writes the latest known fix into *out. Returns true if at least
 * one field was updated during this call. Call this from sensor_task at
 * the task's normal rate; GPS itself only updates at ~1-10 Hz so most
 * calls will simply report "no update". */
bool gps_nmea_poll(gps_nmea_t *gps, fc_gps_sample_t *out, TickType_t timeout_ticks);

#ifdef __cplusplus
}
#endif
