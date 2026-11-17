#pragma once

#include <stddef.h>
#include <string.h>

namespace observer {

// Independent one-entry host command slot; ACK traffic has its own slot.
class CommandIngress {
 public:
  static constexpr size_t kCapacity = 96;

  bool submitCommand(const char* bytes, size_t length) {
    if (!bytes || length == 0 || length >= kCapacity || memchr(bytes, 0, length) ||
        __atomic_load_n(&length_, __ATOMIC_ACQUIRE) != 0) return false;
    memcpy(bytes_, bytes, length);
    bytes_[length] = 0;
    __atomic_store_n(&length_, length, __ATOMIC_RELEASE);
    return true;
  }

  bool takeCommand(char* destination, size_t capacity) {
    const size_t pending = __atomic_load_n(&length_, __ATOMIC_ACQUIRE);
    if (!destination || pending == 0 || capacity <= pending) return false;
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
