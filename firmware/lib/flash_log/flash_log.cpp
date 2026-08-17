#include "flash_log.h"

#include <esp32-hal-log.h>

bool FlashLog::begin(bool erase_before_flight) {
    part_ = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40,
                                      "flight_log");
    if (part_ == nullptr) {
        log_e("flash_log: flight_log partition not found -- check partitions.csv");
        return false;
    }
    write_offset_ = 0;

    if (erase_before_flight) {
        log_i("flash_log: erasing flight_log partition (%u bytes)...", (unsigned)part_->size);
        if (esp_partition_erase_range(part_, 0, part_->size) != ESP_OK) {
            return false;
        }
    }
    return true;
}

bool FlashLog::write(const uint8_t *frame, size_t frame_len) {
    if (write_offset_ + frame_len > part_->size) {
        return false;
    }
    if (esp_partition_write(part_, write_offset_, frame, frame_len) != ESP_OK) {
        return false;
    }
    write_offset_ += frame_len;
    return true;
}

bool FlashLog::read(size_t offset, void *out, size_t len) const {
    if (offset + len > part_->size) {
        return false;
    }
    return esp_partition_read(part_, offset, out, len) == ESP_OK;
}
