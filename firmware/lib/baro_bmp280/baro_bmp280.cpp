#include "baro_bmp280.h"

#include <math.h>

bool BaroBmp280::begin(uint8_t addr) {
    if (!bmp_.begin(addr)) {
        began_ = false;
        return false;
    }
    /* osrs_t=x1, osrs_p=x4, filter=x4, standby as short as the library
     * exposes (STANDBY_MS_1 -- despite the name this maps to the BMP280's
     * fastest 0.5ms standby setting) -- pressure resolution matters more
     * than temperature for an altimeter; same tuning as the earlier
     * hand-rolled register-level driver (see docs/ARCHITECTURE.md). */
    bmp_.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X1,
                      Adafruit_BMP280::SAMPLING_X4, Adafruit_BMP280::FILTER_X4,
                      Adafruit_BMP280::STANDBY_MS_1);
    began_ = true;
    return true;
}

bool BaroBmp280::read(fc_baro_sample_t *out) {
    if (!began_) {
        return false;
    }
    out->pressure_pa = bmp_.readPressure();
    out->temperature_c = bmp_.readTemperature();
    return !isnan(out->pressure_pa) && !isnan(out->temperature_c);
}
