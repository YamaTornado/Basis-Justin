#include "gps_m100.h"

bool GpsM100::begin(HardwareSerial &serial, int rx_gpio, int tx_gpio, int baud) {
    serial_ = &serial;
    serial_->begin(baud, SERIAL_8N1, rx_gpio, tx_gpio);
    return true;
}

bool GpsM100::poll(fc_gps_sample_t *out) {
    bool updated = false;

    while (serial_->available() > 0) {
        if (gps_.encode(serial_->read())) {
            /* encode() returned true: a complete, checksum-valid NMEA
             * sentence was just consumed. Pull whatever fields are valid
             * right now -- TinyGPS++ merges data across GGA/RMC/etc
             * sentences internally. */
            updated = true;
            last_.timestamp_us = (int64_t)micros();

            if (gps_.location.isValid()) {
                last_.latitude_deg = gps_.location.lat();
                last_.longitude_deg = gps_.location.lng();
                /* TinyGPS++ doesn't expose a 2D/3D fix type distinction the
                 * way raw GGA fix-quality does -- approximate: 3D if
                 * altitude is also valid, else 2D. */
                last_.fix_type = gps_.altitude.isValid() ? 2 : 1;
            } else {
                last_.fix_type = 0;
            }
            if (gps_.altitude.isValid()) {
                last_.altitude_m = (float)gps_.altitude.meters();
            }
            if (gps_.speed.isValid()) {
                last_.speed_mps = (float)gps_.speed.mps();
            }
            if (gps_.satellites.isValid()) {
                last_.num_sats = (uint8_t)gps_.satellites.value();
            }
        }
    }

    *out = last_;
    return updated;
}
