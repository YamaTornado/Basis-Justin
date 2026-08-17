#include "flash_log.h"

#include "esp_log.h"

static const char *TAG = "flash_log";

esp_err_t flash_log_init(flash_log_t *log, bool erase_before_flight) {
    log->part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x40, "flight_log");
    if (log->part == NULL) {
        ESP_LOGE(TAG, "flight_log partition not found -- check partitions.csv");
        return ESP_ERR_NOT_FOUND;
    }
    log->write_offset = 0;

    if (erase_before_flight) {
        ESP_LOGI(TAG, "erasing flight_log partition (%u bytes)...", (unsigned)log->part->size);
        esp_err_t err = esp_partition_erase_range(log->part, 0, log->part->size);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t flash_log_write(flash_log_t *log, const uint8_t *frame, size_t frame_len) {
    if (log->write_offset + frame_len > log->part->size) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = esp_partition_write(log->part, log->write_offset, frame, frame_len);
    if (err != ESP_OK) {
        return err;
    }
    log->write_offset += frame_len;
    return ESP_OK;
}

size_t flash_log_bytes_written(const flash_log_t *log) {
    return log->write_offset;
}

esp_err_t flash_log_read(const flash_log_t *log, size_t offset, void *out, size_t len) {
    if (offset + len > log->part->size) {
        return ESP_ERR_INVALID_SIZE;
    }
    return esp_partition_read(log->part, offset, out, len);
}
