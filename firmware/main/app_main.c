#include "bmp280.h"
#include "esp_log.h"
#include "flash_log.h"
#include "flight_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps_nmea.h"
#include "i2c_bus.h"
#include "logger_task.h"
#include "lora_lr02.h"
#include "lsm9ds1.h"
#include "nvs_flash.h"
#include "pin_config.h"
#include "queues.h"
#include "sensor_task.h"
#include "telemetry_task.h"

static const char *TAG = "app_main";

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
 * program -- there is exactly one instance of each, created once here. */
static i2c_master_bus_handle_t s_i2c_bus;
static lsm9ds1_t s_imu;
static bmp280_t s_baro;
static gps_nmea_t s_gps;
static lora_lr02_t s_lora;
static flash_log_t s_flash_log;
static fc_queues_t s_queues;

static sensor_task_config_t s_sensor_cfg;
static estimator_task_config_t s_estimator_cfg;
static flight_task_config_t s_flight_cfg;
static logger_task_config_t s_logger_cfg;
static telemetry_task_config_t s_telemetry_cfg;

static void init_nvs(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void init_sensors(void) {
    i2c_bus_config_t i2c_cfg = {
        .sda = PIN_I2C_SDA,
        .scl = PIN_I2C_SCL,
        .clk_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_bus_init(&i2c_cfg, &s_i2c_bus));

    lsm9ds1_config_t imu_cfg;
    lsm9ds1_config_default(&imu_cfg);
    ESP_ERROR_CHECK(lsm9ds1_init(&s_imu, s_i2c_bus, &imu_cfg));
    ESP_LOGI(TAG, "LSM9DS1 initialized");

    ESP_ERROR_CHECK(bmp280_init(&s_baro, s_i2c_bus, BMP280_ADDR_DEFAULT));
    ESP_LOGI(TAG, "BMP280 initialized");

    gps_nmea_config_t gps_cfg;
    gps_nmea_config_default(&gps_cfg, UART_NUM_1, PIN_GPS_UART_TX, PIN_GPS_UART_RX);
    ESP_ERROR_CHECK(gps_nmea_init(&s_gps, &gps_cfg));
    ESP_LOGI(TAG, "GPS UART initialized");
}

static void init_lora(void) {
    lora_lr02_config_t lora_cfg;
    lora_lr02_config_default(&lora_cfg, UART_NUM_2, PIN_LORA_UART_TX, PIN_LORA_UART_RX);
    esp_err_t err = lora_lr02_init(&s_lora, &lora_cfg);
    if (err != ESP_OK) {
        /* Non-fatal: telemetry is a "nice to have" per docs/MISSION_GOALS.md
         * ("der Link darf ausfallen, ohne dass Logging oder Deployment davon
         * abhängen") -- keep booting even if the LoRa module doesn't answer. */
        ESP_LOGE(TAG, "LoRa init failed (err=%d) -- continuing without telemetry", err);
    } else {
        ESP_LOGI(TAG, "LoRa initialized");
    }
}

void app_main(void) {
    init_nvs();
    init_sensors();
    init_lora();

    ESP_ERROR_CHECK(flash_log_init(&s_flash_log, /*erase_before_flight=*/true));
    ESP_ERROR_CHECK(fc_queues_create(&s_queues));

    s_sensor_cfg = (sensor_task_config_t){
        .imu = &s_imu, .baro = &s_baro, .gps = &s_gps, .queues = &s_queues,
        .rate_hz = SENSOR_TASK_RATE_HZ,
    };
    s_estimator_cfg = (estimator_task_config_t){.queues = &s_queues};

    s_flight_cfg = (flight_task_config_t){.queues = &s_queues};
    s_flight_cfg.drogue_pyro_gpio = PIN_PYRO_DROGUE;
    s_flight_cfg.main_pyro_gpio = PIN_PYRO_MAIN;
    s_flight_cfg.arm_switch_gpio = PIN_ARM_SWITCH;
    fc_flight_state_config_default(&s_flight_cfg.state_cfg);

    s_logger_cfg = (logger_task_config_t){
        .queues = &s_queues, .log = &s_flash_log, .rate_hz = LOGGER_TASK_RATE_HZ,
    };
    s_telemetry_cfg = (telemetry_task_config_t){
        .queues = &s_queues, .lora = &s_lora, .rate_hz = TELEMETRY_TASK_RATE_HZ,
    };

    xTaskCreate(sensor_task, "sensor", 4096, &s_sensor_cfg, PRIO_SENSOR, NULL);
    xTaskCreate(estimator_task, "estimator", 4096, &s_estimator_cfg, PRIO_ESTIMATOR, NULL);
    xTaskCreate(flight_task, "flight", 4096, &s_flight_cfg, PRIO_FLIGHT, NULL);
    xTaskCreate(logger_task, "logger", 4096, &s_logger_cfg, PRIO_LOGGER, NULL);
    xTaskCreate(telemetry_task, "telemetry", 4096, &s_telemetry_cfg, PRIO_TELEMETRY, NULL);

    ESP_LOGI(TAG, "flight computer up, all tasks started");
}
