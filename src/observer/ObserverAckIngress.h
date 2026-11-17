#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ObserverProtocol.h"

namespace observer {

// One pending ACK is sufficient for stop-and-wait. A second write is rejected
// until the main loop drains the slot; it can never overwrite a host command.
class AckIngress {
 public:
  static constexpr size_t kCapacity = 95;

  bool submit(const char* bytes, size_t length) {
    Ack parsed{};
    if (!bytes || length == 0 || length >= kCapacity ||
        __atomic_load_n(&length_, __ATOMIC_ACQUIRE) != 0 ||
        !parseAck(bytes, length, parsed)) return false;
    memcpy(bytes_, bytes, length);
    bytes_[length] = 0;
    __atomic_store_n(&length_, length, __ATOMIC_RELEASE);
    return true;
  }

  bool take(char* destination, size_t capacity, size_t& length) {
    const size_t pending = __atomic_load_n(&length_, __ATOMIC_ACQUIRE);
    if (!destination || capacity <= pending || pending == 0) return false;
    length = pending;
    memcpy(destination, bytes_, pending + 1);
    __atomic_store_n(&length_, 0, __ATOMIC_RELEASE);
    return true;
  }

  void clear() { __atomic_store_n(&length_, 0, __ATOMIC_RELEASE); }

 private:
  char bytes_[kCapacity]{};
  size_t length_ = 0;
};

}  // namespace observer
