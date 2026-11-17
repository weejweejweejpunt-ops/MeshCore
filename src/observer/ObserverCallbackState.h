#pragma once

#include <stdint.h>

namespace observer {

// One-slot atomic handoff. Callback tasks publish; the Arduino loop consumes.
class CallbackEvents {
 public:
  bool postConnect(uint16_t handle) { return publish(connect_handle_, connect_state_, handle); }
  bool postDisconnect(uint16_t handle) { return publish(disconnect_handle_, disconnect_state_, handle); }
  bool postSecured(uint16_t handle) { return publish(secured_handle_, secured_state_, handle); }
  bool postPairComplete(uint16_t handle, uint8_t status) {
    if (!claim(pair_state_)) return false;
    __atomic_store_n(&pair_handle_, handle, __ATOMIC_RELAXED);
    __atomic_store_n(&pair_status_, status, __ATOMIC_RELAXED);
    __atomic_store_n(&pair_state_, kReady, __ATOMIC_RELEASE);
    return true;
  }

  bool takeConnect(uint16_t& handle) { return take(connect_handle_, connect_state_, handle); }
  bool takeDisconnect(uint16_t& handle) { return take(disconnect_handle_, disconnect_state_, handle); }
  bool takeSecured(uint16_t& handle) { return take(secured_handle_, secured_state_, handle); }
  bool takePairComplete(uint16_t& handle, uint8_t& status) {
    if (!claimReady(pair_state_)) return false;
    handle = __atomic_load_n(&pair_handle_, __ATOMIC_RELAXED);
    status = __atomic_load_n(&pair_status_, __ATOMIC_RELAXED);
    __atomic_store_n(&pair_state_, kEmpty, __ATOMIC_RELEASE);
    return true;
  }

  bool pending() const {
    return __atomic_load_n(&connect_state_, __ATOMIC_ACQUIRE) != kEmpty ||
           __atomic_load_n(&disconnect_state_, __ATOMIC_ACQUIRE) != kEmpty ||
           __atomic_load_n(&secured_state_, __ATOMIC_ACQUIRE) != kEmpty ||
           __atomic_load_n(&pair_state_, __ATOMIC_ACQUIRE) != kEmpty;
  }
  bool safeToReleaseQueue(bool enabled, uint16_t active_handle) const {
    return !enabled && active_handle == kInvalidHandle && !pending();
  }
  bool safeToReleaseQueue(bool enabled, uint16_t active_handle, bool admission_pending) const {
    return !enabled && active_handle == kInvalidHandle && !pending() && !admission_pending;
  }

 private:
  static constexpr uint8_t kEmpty = 0;
  static constexpr uint8_t kWriting = 1;
  static constexpr uint8_t kReady = 2;
  static constexpr uint16_t kInvalidHandle = 0xFFFF;

  static bool claim(volatile uint8_t& state) {
    uint8_t expected = kEmpty;
    return __atomic_compare_exchange_n(&state, &expected, kWriting, false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED);
  }
  static bool claimReady(volatile uint8_t& state) {
    uint8_t expected = kReady;
    return __atomic_compare_exchange_n(&state, &expected, kWriting, false, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED);
  }
  static bool publish(volatile uint16_t& slot, volatile uint8_t& state, uint16_t handle) {
    if (!claim(state)) return false;
    __atomic_store_n(&slot, handle, __ATOMIC_RELAXED);
    __atomic_store_n(&state, kReady, __ATOMIC_RELEASE);
    return true;
  }
  static bool take(volatile uint16_t& slot, volatile uint8_t& state, uint16_t& handle) {
    if (!claimReady(state)) return false;
    handle = __atomic_load_n(&slot, __ATOMIC_RELAXED);
    __atomic_store_n(&state, kEmpty, __ATOMIC_RELEASE);
    return true;
  }

  volatile uint16_t connect_handle_ = kInvalidHandle;
  volatile uint16_t disconnect_handle_ = kInvalidHandle;
  volatile uint16_t secured_handle_ = kInvalidHandle;
  volatile uint16_t pair_handle_ = kInvalidHandle;
  volatile uint8_t pair_status_ = 0;
  volatile uint8_t connect_state_ = kEmpty;
  volatile uint8_t disconnect_state_ = kEmpty;
  volatile uint8_t secured_state_ = kEmpty;
  volatile uint8_t pair_state_ = kEmpty;
};

}  // namespace observer
