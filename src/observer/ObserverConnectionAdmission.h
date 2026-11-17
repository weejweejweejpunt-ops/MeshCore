#pragma once

#include <stdint.h>

#include "ObserverPolicy.h"

namespace observer {

// Stable public/static addresses are directly comparable at connection time;
// resolved RPAs carry the identity address according to S140, while unresolved
// private addresses must wait for the secured bond identity under the timeout.
inline bool identityKnownAtConnect(bool bonded, uint8_t address_type, uint8_t address_id_peer) {
  return bonded || address_id_peer != 0 || address_type == 0 || address_type == 1;
}

inline bool hasLiveConnection(bool connection_exists, bool connected) {
  return connection_exists && connected;
}

// Decides whether a freshly connected handle may occupy the single peripheral slot.
inline bool admitsCandidate(bool enabled, bool slot_free, bool link_connected,
                            bool enrolling, bool enrollment_slot_free,
                            bool has_pinned, bool identity_known,
                            const PeerIdentity& observed, const PeerIdentity& pinned) {
  if (!enabled || !slot_free || !link_connected) return false;
  if (enrolling) return enrollment_slot_free;
  if (!has_pinned) return false;
  if (identity_known) return samePeer(observed, pinned);
  return true;
}

class ConnectionAdmission {
 public:
  explicit ConnectionAdmission(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}
  bool connected(uint16_t handle, uint32_t now) {
    if (handle_ != kInvalidHandle) return false;
    handle_ = handle;
    connected_at_ = now;
    authorized_ = false;
    return true;
  }
  bool authorize(uint16_t handle, const PeerIdentity& peer, const PeerIdentity& intended) {
    if (handle != handle_ || !samePeer(peer, intended)) return false;
    authorized_ = true;
    return true;
  }
  bool expired(uint32_t now) const {
    return handle_ != kInvalidHandle && !authorized_ &&
           static_cast<uint32_t>(now - connected_at_) >= timeout_ms_;
  }
  bool takeDisconnect(uint32_t now, uint16_t& handle) {
    if (!expired(now)) return false;
    handle = handle_;
    reset();
    return true;
  }
  void disconnected(uint16_t handle) { if (handle_ == handle) reset(); }
  bool authorized() const { return authorized_; }
  bool pending() const { return handle_ != kInvalidHandle && !authorized_; }
 private:
  static constexpr uint16_t kInvalidHandle = 0xFFFF;
  void reset() { handle_ = kInvalidHandle; connected_at_ = 0; authorized_ = false; }
  uint32_t timeout_ms_;
  uint32_t connected_at_ = 0;
  uint16_t handle_ = kInvalidHandle;
  bool authorized_ = false;
};

}  // namespace observer
