#pragma once

#include <stdio.h>
#include <stdint.h>

namespace observer {

struct StatsValues {
  uint32_t evicted_records = 0;
  uint32_t evicted_bytes = 0;
  uint32_t truncated_records = 0;
  uint32_t truncated_bytes = 0;
  uint32_t failed_notifications = 0;
  uint32_t unavailable_records = 0;
  uint32_t unavailable_bytes = 0;
};

inline size_t formatStats(const StatsValues& stats, char* buffer, size_t capacity) {
  const int length = snprintf(buffer, capacity, "evicted=%lu/%lu truncated=%lu/%lu failed=%lu unavailable=%lu/%lu",
      static_cast<unsigned long>(stats.evicted_records), static_cast<unsigned long>(stats.evicted_bytes),
      static_cast<unsigned long>(stats.truncated_records), static_cast<unsigned long>(stats.truncated_bytes),
      static_cast<unsigned long>(stats.failed_notifications),
      static_cast<unsigned long>(stats.unavailable_records), static_cast<unsigned long>(stats.unavailable_bytes));
  return length < 0 ? 0 : static_cast<size_t>(length);
}

// Single loop-task writer, callback-task reader. The writer fills the inactive
// buffer and then release-publishes the active index; the reader acquire-loads the
// index and copies a buffer the writer is not touching. The reader never spins, so
// the higher-priority BLE callback cannot livelock the loop task mid-publish.
class StatsSnapshot {
 public:
  void publish(uint32_t evicted_records, uint32_t evicted_bytes,
               uint32_t truncated_records, uint32_t truncated_bytes,
               uint32_t failed_notifications, uint32_t unavailable_records = 0,
               uint32_t unavailable_bytes = 0) {
    const uint32_t active = __atomic_load_n(&active_, __ATOMIC_ACQUIRE);
    StatsValues& back = buffers_[active ^ 1U];
    back.evicted_records = evicted_records;
    back.evicted_bytes = evicted_bytes;
    back.truncated_records = truncated_records;
    back.truncated_bytes = truncated_bytes;
    back.failed_notifications = failed_notifications;
    back.unavailable_records = unavailable_records;
    back.unavailable_bytes = unavailable_bytes;
    __atomic_store_n(&active_, active ^ 1U, __ATOMIC_RELEASE);
  }

  StatsValues read() const {
    const uint32_t active = __atomic_load_n(&active_, __ATOMIC_ACQUIRE);
    return buffers_[active];
  }

 private:
  mutable volatile uint32_t active_ = 0;
  StatsValues buffers_[2]{};
};

}  // namespace observer
