#include <Arduino.h>
#include <Wire.h>
#include <nvs_flash.h>

#include "baro_bmp280.h"
#include "estimator_task.h"
#include "flash_log.h"
#include "flight_task.h"
#include "gps_m100.h"
#include "imu_lsm9ds1.h"
#include "logger_task.h"
#include "lora_lr02.h"
#include "pin_config.h"
#include "queues.h"
#include "sensor_task.h"
#include "telemetry_task.h"

/* Task rates and priorities per docs/ARCHITECTURE.md "Datenfluss
 * (Firmware)" table. Priority spacing leaves room to insert tasks later
 * without renumbering everything. */
#define SENSOR_TASK_RATE_HZ 100
#define LOGGER_TASK_RATE_HZ 50
#define TELEMETRY_TASK_RATE_HZ 8

#define PRIO_FLIGHT 10   /* highest: deployment must never be late */
#define PRIO_SENSOR 9
#define PRIO_ESTIMATOR 9
#define PRIO_LOGGER 7
#define PRIO_TELEMETRY 5 /* lowest: a stalled LoRa send must never back up anything else */

/* All of these are static (not stack/heap-transient) because the FreeRTOS
 * tasks they're passed to keep referencing them for the lifetime of the
 * program -- there is exactly one instance of each, created once in
 * setup(). */
static ImuLsm9ds1 s_imu;
static BaroBmp280 s_baro;
static GpsM100 s_gps;
static LoraLr02 s_lora;
static FlashLog s_flash_log;
static FcQueues s_queues;

static SensorTaskConfig s_sensor_cfg;
static EstimatorTaskConfig s_estimator_cfg;
static FlightTaskConfig s_flight_cfg;
static LoggerTaskConfig s_logger_cfg;
static TelemetryTaskConfig s_telemetry_cfg;

static void init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        log_e("main: nvs_flash_init failed: %d", err);
    }
}

static void init_sensors() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000U);

    if (!s_imu.begin()) {
        log_e("main: LSM9DS1 init failed -- check wiring/I2C address");
    } else {
        log_i("main: LSM9DS1 initialized");
    }

    if (!s_baro.begin()) {
        log_e("main: BMP280 init failed -- check wiring/I2C address");
    } else {
        log_i("main: BMP280 initialized");
    }

    s_gps.begin(Serial1, PIN_GPS_UART_RX, PIN_GPS_UART_TX);
    log_i("main: GPS UART initialized");
}

static void init_lora() {
    /* Non-fatal if this fails: telemetry is a "nice to have" per
     * docs/MISSION_GOALS.md ("der Link darf ausfallen, ohne dass Logging
     * oder Deployment davon abhängen") -- keep booting either way. */
    if (!s_lora.begin(Serial2, PIN_LORA_UART_RX, PIN_LORA_UART_TX)) {
        log_e("main: LoRa init failed -- continuing without telemetry");
    } else {
        log_i("main: LoRa initialized");
    }
}

void setup() {
    Serial.begin(115200);

    init_nvs();
    init_sensors();
    init_lora();

    if (!s_flash_log.begin(/*erase_before_flight=*/true)) {
        log_e("main: flash_log init failed -- logging will be unavailable");
    }
    if (!fc_queues_create(&s_queues)) {
        log_e("main: queue creation failed -- halting");
        while (true) {
            delay(1000);
        }
    }

    s_sensor_cfg.imu = &s_imu;
    s_sensor_cfg.baro = &s_baro;
    s_sensor_cfg.gps = &s_gps;
    s_sensor_cfg.queues = &s_queues;
    s_sensor_cfg.rate_hz = SENSOR_TASK_RATE_HZ;

    s_estimator_cfg.queues = &s_queues;

    s_flight_cfg.queues = &s_queues;
    s_flight_cfg.drogue_pyro_gpio = PIN_PYRO_DROGUE;
    s_flight_cfg.main_pyro_gpio = PIN_PYRO_MAIN;
    s_flight_cfg.arm_switch_gpio = PIN_ARM_SWITCH;
    fc_flight_state_config_default(&s_flight_cfg.state_cfg);

    s_logger_cfg.queues = &s_queues;
    s_logger_cfg.log = &s_flash_log;
    s_logger_cfg.rate_hz = LOGGER_TASK_RATE_HZ;

    s_telemetry_cfg.queues = &s_queues;
    s_telemetry_cfg.lora = &s_lora;
    s_telemetry_cfg.rate_hz = TELEMETRY_TASK_RATE_HZ;

    xTaskCreate(sensor_task, "sensor", 4096, &s_sensor_cfg, PRIO_SENSOR, nullptr);
    xTaskCreate(estimator_task, "estimator", 4096, &s_estimator_cfg, PRIO_ESTIMATOR, nullptr);
    xTaskCreate(flight_task, "flight", 4096, &s_flight_cfg, PRIO_FLIGHT, nullptr);
    xTaskCreate(logger_task, "logger", 4096, &s_logger_cfg, PRIO_LOGGER, nullptr);
    xTaskCreate(telemetry_task, "telemetry", 4096, &s_telemetry_cfg, PRIO_TELEMETRY, nullptr);

    log_i("main: flight computer up, all tasks started");

    /* All real work happens in the tasks created above; nothing left for
     * Arduino's own loop task to do. */
    vTaskDelete(nullptr);
}

void loop() {
    /* unreachable -- setup() deletes the loop task */
}
