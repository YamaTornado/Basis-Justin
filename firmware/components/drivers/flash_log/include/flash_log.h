#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_partition.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Writes fc_log_format frames into the raw "flight_log" data partition
 * (see firmware/partitions.csv). v1 is deliberately simple: linear
 * append-only log from the start of the partition, no wraparound. The
 * partition is fully erased at init (erase_before_flight=true) as a
 * "arm/reset" step -- ground procedure is: connect, erase, arm, fly. This
 * avoids the complexity of a resumable ring buffer for the first version;
 * revisit if flights regularly exceed the partition size (see
 * docs/ARCHITECTURE.md for the size budget). Once full, writes fail with
 * ESP_ERR_NO_MEM rather than wrapping and overwriting earlier data. */

typedef struct {
    const esp_partition_t *part;
    size_t write_offset;
} flash_log_t;

esp_err_t flash_log_init(flash_log_t *log, bool erase_before_flight);

/* Appends one already-framed record (as produced by fc_log_encode_*()) to
 * the log. Returns ESP_ERR_NO_MEM if the partition is full. */
esp_err_t flash_log_write(flash_log_t *log, const uint8_t *frame, size_t frame_len);

/* How many bytes have been written this session -- the valid range for
 * flash_log_read() is [0, flash_log_bytes_written()). */
size_t flash_log_bytes_written(const flash_log_t *log);

/* Raw read for post-flight retrieval (e.g. a future ground command that
 * dumps the log over LoRa/USB for tools/log_dump to decode with the same
 * fc_log_decode_next() the firmware uses). */
esp_err_t flash_log_read(const flash_log_t *log, size_t offset, void *out, size_t len);

#ifdef __cplusplus
}
#endif
