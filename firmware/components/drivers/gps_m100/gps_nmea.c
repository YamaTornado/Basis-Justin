#include "gps_nmea.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "gps_nmea";

#define UART_RX_BUF_SIZE 512

void gps_nmea_config_default(gps_nmea_config_t *cfg, uart_port_t port, int tx_gpio, int rx_gpio) {
    cfg->uart_port = port;
    cfg->tx_gpio = tx_gpio;
    cfg->rx_gpio = rx_gpio;
    cfg->baud_rate = 9600;
}

esp_err_t gps_nmea_init(gps_nmea_t *gps, const gps_nmea_config_t *cfg) {
    memset(gps, 0, sizeof(*gps));
    gps->uart_port = cfg->uart_port;

    uart_config_t uart_cfg = {
        .baud_rate = cfg->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config(cfg->uart_port, &uart_cfg);
    if (err != ESP_OK) return err;
    err = uart_set_pin(cfg->uart_port, cfg->tx_gpio, cfg->rx_gpio, UART_PIN_NO_CHANGE,
                        UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;
    return uart_driver_install(cfg->uart_port, UART_RX_BUF_SIZE, 0, 0, NULL, 0);
}

/* NMEA lat/lon are ddmm.mmmm (lon: dddmm.mmmm) -- degrees + decimal minutes.
 * Both lat (2-digit degrees) and lon (3-digit degrees) parse the same way:
 * whole-number part above the last two integer digits is degrees, the rest
 * (including the decimal) is minutes. */
static double parse_lat_lon(const char *field) {
    if (field[0] == '\0') return 0.0;
    double raw = atof(field);
    long degrees = (long)(raw / 100.0);
    double minutes = raw - (double)(degrees * 100);
    return (double)degrees + minutes / 60.0;
}

/* Splits a comma-separated NMEA sentence in-place, returns field count.
 * fields[i] point into line (NUL-terminated by overwriting commas). Strips
 * the trailing "*checksum" from the last field. */
static int split_fields(char *line, char *fields[], int max_fields) {
    int n = 0;
    char *star = strchr(line, '*');
    if (star) *star = '\0';
    char *p = line;
    while (n < max_fields) {
        fields[n++] = p;
        char *comma = strchr(p, ',');
        if (!comma) break;
        *comma = '\0';
        p = comma + 1;
    }
    return n;
}

static bool parse_gga(char *line, fc_gps_sample_t *s) {
    char *f[16] = {0};
    int n = split_fields(line, f, 16);
    if (n < 10) return false;
    /* f[0]=$..GGA f[1]=time f[2]=lat f[3]=N/S f[4]=lon f[5]=E/W f[6]=fix
     * f[7]=numSV f[8]=hdop f[9]=altitude f[10]="M" */
    if (f[6][0] != '\0') {
        int fix_quality = atoi(f[6]);
        s->fix_type = fix_quality > 0 ? 2 : 0; /* GGA doesn't distinguish 2D/3D */
    }
    if (f[7][0] != '\0') s->num_sats = (uint8_t)atoi(f[7]);
    if (f[2][0] != '\0' && f[3][0] != '\0') {
        double lat = parse_lat_lon(f[2]);
        if (f[3][0] == 'S') lat = -lat;
        s->latitude_deg = lat;
    }
    if (f[4][0] != '\0' && f[5][0] != '\0') {
        double lon = parse_lat_lon(f[4]);
        if (f[5][0] == 'W') lon = -lon;
        s->longitude_deg = lon;
    }
    if (n > 9 && f[9][0] != '\0') s->altitude_m = (float)atof(f[9]);
    return true;
}

static bool parse_rmc(char *line, fc_gps_sample_t *s) {
    char *f[13] = {0};
    int n = split_fields(line, f, 13);
    if (n < 8) return false;
    /* f[0]=$..RMC f[1]=time f[2]=status(A/V) f[3]=lat f[4]=N/S f[5]=lon
     * f[6]=E/W f[7]=speed(knots) f[8]=course */
    if (f[2][0] == 'V') {
        s->fix_type = 0; /* void / no fix */
    }
    if (f[7][0] != '\0') {
        float knots = (float)atof(f[7]);
        s->speed_mps = knots * 0.514444f;
    }
    return true;
}

bool gps_nmea_poll(gps_nmea_t *gps, fc_gps_sample_t *out, TickType_t timeout_ticks) {
    uint8_t chunk[128];
    int n = uart_read_bytes(gps->uart_port, chunk, sizeof(chunk), timeout_ticks);
    bool updated = false;

    for (int i = 0; i < n; i++) {
        char c = (char)chunk[i];
        if (c == '\n' || c == '\r') {
            if (gps->line_len > 6) {
                gps->line_buf[gps->line_len] = '\0';
                gps->last.timestamp_us = esp_timer_get_time();
                if (strstr(gps->line_buf, "GGA") != NULL) {
                    updated |= parse_gga(gps->line_buf, &gps->last);
                } else if (strstr(gps->line_buf, "RMC") != NULL) {
                    updated |= parse_rmc(gps->line_buf, &gps->last);
                }
            }
            gps->line_len = 0;
            continue;
        }
        if (gps->line_len < sizeof(gps->line_buf) - 1) {
            gps->line_buf[gps->line_len++] = c;
        } else {
            /* line too long (corrupt/unsupported sentence) -- drop it */
            gps->line_len = 0;
        }
    }

    *out = gps->last;
    return updated;
}
