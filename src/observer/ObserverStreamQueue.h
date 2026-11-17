#pragma once

#include <stddef.h>
#include <stdint.h>

namespace observer {

template <size_t Capacity>
class StreamQueue {
 public:
  explicit StreamQueue(bool enabled = true) : enabled_(enabled) {}

  size_t enqueue(const uint8_t* bytes, size_t length) {
    if (!enabled_) return length;
    if (length > available()) {
      ++dropped_records_;
      dropped_bytes_ += length;
      return 0;
    }
    for (size_t index = 0; index < length; ++index) {
      bytes_[tail_] = bytes[index];
      tail_ = (tail_ + 1) % Capacity;
    }
    size_ += length;
    return length;
  }

  size_t dequeue(uint8_t* destination, size_t maximum) {
    const size_t copied = maximum < size_ ? maximum : size_;
    for (size_t index = 0; index < copied; ++index) {
      destination[index] = bytes_[head_];
      head_ = (head_ + 1) % Capacity;
    }
    size_ -= copied;
    return copied;
  }

  void drop(size_t length) {
    ++dropped_records_;
    dropped_bytes_ += length;
  }
  bool empty() const { return size_ == 0; }
  size_t available() const { return Capacity - size_; }
  uint32_t droppedRecords() const { return dropped_records_; }
  uint32_t droppedBytes() const { return dropped_bytes_; }

 private:
  uint8_t bytes_[Capacity]{};
  size_t head_ = 0;
  size_t tail_ = 0;
  size_t size_ = 0;
  bool enabled_;
  uint32_t dropped_records_ = 0;
  uint32_t dropped_bytes_ = 0;
};

}  // namespace observer
