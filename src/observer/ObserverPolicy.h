#pragma once

#include <stdint.h>
#include <string.h>

namespace observer {

struct PeerIdentity {
  uint8_t type = 0;
  uint8_t address[6]{};
};

inline bool samePeer(const PeerIdentity& left, const PeerIdentity& right) {
  return left.type == right.type && memcmp(left.address, right.address, sizeof(left.address)) == 0;
}

class Policy {
 public:
  Policy(bool enabled = false, const PeerIdentity& peer = PeerIdentity{}, bool has_peer = false)
      : enabled_(enabled), peer_(peer), has_peer_(has_peer) {}

  bool enabled() const { return enabled_; }
  void setEnabled(bool enabled) { enabled_ = enabled; }
  bool hasPeer() const { return has_peer_; }
  const PeerIdentity& peer() const { return peer_; }

  void openEnrollment(uint32_t now, uint32_t duration_ms) {
    if (duration_ms == 0 || duration_ms > kMaximumEnrollmentMs || has_peer_) return;
    enrollment_started_ = now;
    enrollment_duration_ = duration_ms;
    enrollment_open_ = true;
  }

  void closeEnrollment() { enrollment_open_ = false; }

  bool enrollmentOpen(uint32_t now) const {
    return enrollment_open_ && static_cast<uint32_t>(now - enrollment_started_) < enrollment_duration_;
  }

  bool enroll(const PeerIdentity& peer, uint32_t now) {
    if (!enabled_ || !enrollmentOpen(now) || has_peer_) return false;
    peer_ = peer;
    has_peer_ = true;
    enrollment_open_ = false;
    return true;
  }

  bool restorePeer(const PeerIdentity& peer) {
    peer_ = peer;
    has_peer_ = true;
    return true;
  }

  bool authorizes(bool connected, const PeerIdentity& peer, uint32_t now = 0) const {
    return enabled_ && has_peer_ && connected && !enrollmentOpen(now) &&
           peer.type == peer_.type &&
           memcmp(peer.address, peer_.address, sizeof(peer.address)) == 0;
  }

  void revoke() {
    has_peer_ = false;
    memset(&peer_, 0, sizeof(peer_));
    closeEnrollment();
  }

  static constexpr uint32_t kMaximumEnrollmentMs = 60000;

 private:
  bool enabled_ = false;
  PeerIdentity peer_{};
  bool has_peer_ = false;
  bool enrollment_open_ = false;
  uint32_t enrollment_started_ = 0;
  uint32_t enrollment_duration_ = 0;
};

}  // namespace observer
