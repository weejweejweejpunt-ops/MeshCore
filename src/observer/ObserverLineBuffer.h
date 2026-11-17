#pragma once

#include <stddef.h>
#include <stdint.h>

namespace observer {

template <size_t LineCapacity, size_t QueueCapacity, typename Queue = StreamQueue<QueueCapacity>,
          typename OverflowReporter = Queue>
class LineBuffer {
 public:
  explicit LineBuffer(Queue& queue)
      : queue_(queue), overflow_reporter_(nullptr) {}
  LineBuffer(Queue& queue, OverflowReporter& overflow_reporter)
      : queue_(queue), overflow_reporter_(&overflow_reporter) {}

  size_t write(const uint8_t* bytes, size_t length) {
    for (size_t index = 0; index < length; ++index) write(bytes[index]);
    return length;
  }

  size_t write(uint8_t byte) {
    // The byte that exceeds capacity can itself be the terminator. Process it
    // in the discard state immediately, not as the start of another lost line.
    if (line_length_ == LineCapacity) overflowed_ = true;
    if (overflowed_) {
      if (byte == '\n') {
        if (overflow_reporter_) overflow_reporter_->drop(line_length_ + 1);
        else queue_.drop(line_length_ + 1);
        line_length_ = 0;
        overflowed_ = false;
      } else {
        ++line_length_;
      }
      return 1;
    }
    line_[line_length_++] = byte;
    if (byte == '\n') commitLine();
    return 1;
  }

  void reset() {
    line_length_ = 0;
    overflowed_ = false;
  }

 private:
  void commitLine() {
    queue_.enqueue(line_, line_length_);
    line_length_ = 0;
  }

  Queue& queue_;
  OverflowReporter* overflow_reporter_;
  uint8_t line_[LineCapacity]{};
  size_t line_length_ = 0;
  bool overflowed_ = false;
};

}  // namespace observer
