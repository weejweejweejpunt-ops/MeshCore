#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace observer {

constexpr size_t kFrameHeaderBytes = 16;
constexpr size_t kMaximumRecordBytes = 544;
constexpr size_t kMaximumAckBytes = 95;

struct Ack {
  uint64_t boot_nonce = 0;
  uint32_t sequence = 0;
};

inline size_t encodeFrame(uint64_t boot_nonce, uint32_t sequence, uint16_t total_length,
                          uint16_t offset, const uint8_t* bytes, size_t length,
                          uint8_t* output, size_t capacity) {
  if (!output || !bytes || sequence == 0 || total_length == 0 || total_length > kMaximumRecordBytes ||
      offset > total_length || length == 0 || length > total_length - offset ||
      length + kFrameHeaderBytes > capacity) return 0;
  for (unsigned index = 0; index < 8; ++index) output[index] = static_cast<uint8_t>(boot_nonce >> (index * 8));
  for (unsigned index = 0; index < 4; ++index) output[8 + index] = static_cast<uint8_t>(sequence >> (index * 8));
  output[12] = static_cast<uint8_t>(total_length);
  output[13] = static_cast<uint8_t>(total_length >> 8);
  output[14] = static_cast<uint8_t>(offset);
  output[15] = static_cast<uint8_t>(offset >> 8);
  memcpy(output + kFrameHeaderBytes, bytes, length);
  return kFrameHeaderBytes + length;
}

inline bool formatAck(uint64_t boot_nonce, uint32_t sequence, char* output, size_t capacity) {
  if (!output || capacity < 31 || sequence == 0) return false;
  const int length = snprintf(output, capacity, "ack %016llx %u",
                              static_cast<unsigned long long>(boot_nonce), sequence);
  return length > 0 && static_cast<size_t>(length) < capacity;
}

inline bool parseAck(const char* bytes, size_t length, Ack& ack) {
  constexpr char prefix[] = "ack ";
  if (!bytes || length >= kMaximumAckBytes || length < sizeof(prefix) - 1 + 16 + 2 ||
      memcmp(bytes, prefix, sizeof(prefix) - 1) != 0) return false;
  const size_t nonce_start = sizeof(prefix) - 1;
  for (size_t index = 0; index < 16; ++index) {
    const char c = bytes[nonce_start + index];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  }
  const size_t separator = nonce_start + 16;
  if (bytes[separator] != ' ') return false;
  uint64_t nonce = 0;
  for (size_t index = 0; index < 16; ++index) {
    const char c = bytes[nonce_start + index];
    nonce = (nonce << 4) | static_cast<uint64_t>(c <= '9' ? c - '0' : c - 'a' + 10);
  }
  const size_t sequence_start = separator + 1;
  if (sequence_start == length || (bytes[sequence_start] == '0' && sequence_start + 1 != length)) return false;
  uint32_t sequence = 0;
  for (size_t index = sequence_start; index < length; ++index) {
    const char c = bytes[index];
    if (c < '0' || c > '9') return false;
    const uint32_t digit = static_cast<uint32_t>(c - '0');
    if (sequence > (UINT32_MAX - digit) / 10) return false;
    sequence = sequence * 10 + digit;
  }
  if (sequence == 0) return false;
  ack.boot_nonce = nonce;
  ack.sequence = sequence;
  return true;
}

}  // namespace observer
