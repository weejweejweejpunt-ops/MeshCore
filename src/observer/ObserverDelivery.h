#pragma once

#include <stdint.h>

namespace observer {

// Nonblocking stop-and-wait timing state, independent of BLE/Arduino.
class DeliverySchedule {
 public:
  explicit DeliverySchedule(uint32_t retry_interval_ms, uint32_t fragment_interval_ms = 10)
      : retry_interval_ms_(retry_interval_ms), fragment_interval_ms_(fragment_interval_ms) {}

  bool maySend(uint32_t now) const {
    return !waiting_ || static_cast<uint32_t>(now - last_attempt_) >=
                            (complete_ ? retry_interval_ms_ : fragment_interval_ms_);
  }

  void attempted(uint32_t now) {
    last_attempt_ = now;
    waiting_ = true;
  }

  void fragmentSent(bool record_complete) {
    if (record_complete) complete_ = true;
  }

  bool restartWholeRecord() {
    if (!complete_) return false;
    complete_ = false;
    return true;
  }

  void fragmentFailed() { complete_ = false; }
  void disconnected() { waiting_ = false; complete_ = false; }
  void acknowledged() { waiting_ = false; complete_ = false; }

 private:
  uint32_t retry_interval_ms_;
  uint32_t fragment_interval_ms_;
  uint32_t last_attempt_ = 0;
  bool waiting_ = false;
  bool complete_ = false;
};

}  // namespace observer
