#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_partition.h"

/* Writes fc_log_format frames into the raw "flight_log" data partition
 * (see firmware/partitions.csv). v1 is deliberately simple: linear
 * append-only log from the start of the partition, no wraparound. The
 * partition is fully erased at init (erase_before_flight=true) as an
 * "arm/reset" step -- ground procedure is: connect, erase, arm, fly. This
 * avoids the complexity of a resumable ring buffer for the first version;
 * revisit if flights regularly exceed the partition size (see
 * docs/ARCHITECTURE.md for the size budget). Once full, writes fail
 * (write() returns false) rather than wrapping and overwriting earlier
 * data. */
class FlashLog {
   public:
    bool begin(bool erase_before_flight = true);

    /* Appends one already-framed record (as produced by fc_log_encode_*())
     * to the log. Returns false if the partition is full. */
    bool write(const uint8_t *frame, size_t frame_len);

    /* How many bytes have been written this session -- the valid range for
     * read() is [0, bytesWritten()). */
    size_t bytesWritten() const { return write_offset_; }

    /* Raw read for post-flight retrieval (e.g. a future ground command
     * that dumps the log over LoRa/USB for tools/log_dump to decode with
     * the same fc_log_decode_next() the firmware uses). */
    bool read(size_t offset, void *out, size_t len) const;

   private:
    const esp_partition_t *part_ = nullptr;
    size_t write_offset_ = 0;
};
