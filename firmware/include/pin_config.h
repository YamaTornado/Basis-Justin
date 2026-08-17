#pragma once

/* Pin assignments -- board is an ESP32-S3 SuperMini with only the outer
 * header populated/accessible: GPIO 1-13 available, nothing else. GPIO 3 is
 * still avoided (strapping pin, selects JTAG signal source at boot) even
 * though it's in range, to leave one fewer thing to debug if boot gets
 * weird.
 *
 * GPS/BMP280/LoRa (1,2 / 4,5 / 6,7) are already wired -- left untouched.
 * LSM9DS1 (IMU) gets the next two free pins (8,9) on its own I2C bus
 * (Wire1, separate from the BMP280's Wire, see below). Pyro/arm (10-12)
 * are not part of today's bring-up (sensors only for now) but still get
 * pin numbers so nothing collides later; GPIO 13 stays free. */

#define PIN_GPS_UART_RX 1
#define PIN_GPS_UART_TX 2

/* BMP280 (baro) -- own I2C bus (Wire). */
#define PIN_BARO_I2C_SDA 4
#define PIN_BARO_I2C_SCL 5

#define PIN_LORA_UART_RX 6
#define PIN_LORA_UART_TX 7

/* LSM9DS1 (IMU: accel+gyro+mag) -- separate I2C bus (Wire1), so it doesn't
 * share pins with the BMP280. */
#define PIN_IMU_I2C_SDA 8
#define PIN_IMU_I2C_SCL 9

/* Not wired today (sensors-only bring-up) -- reserved so future pyro/arm
 * wiring doesn't collide with anything above. */
#define PIN_PYRO_DROGUE 10
#define PIN_PYRO_MAIN 11
#define PIN_ARM_SWITCH 12
