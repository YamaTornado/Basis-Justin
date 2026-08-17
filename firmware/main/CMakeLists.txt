idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        core
        driver
        log
        nvs_flash
        i2c_bus
        imu_lsm9ds1
        baro_bmp280
        gps_m100
        lora_lr02
        flash_log
        tasks
)
