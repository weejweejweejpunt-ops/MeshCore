#pragma once

#include <helpers/CommonCLI.h>
#include "ObserverPolicy.h"

namespace observer {

class ObserverNodePrefs : public NodePrefs {
 public:
  static constexpr uint8_t kObserverVersion = 1;
  uint8_t observer_enabled = 0;
  uint8_t observer_peer_present = 0;
  uint8_t observer_peer_type = 0;
  uint8_t observer_peer_address[6]{};
  uint8_t observer_version_ = 0;

 protected:
  void structure() override {
    NodePrefs::structure();
    def("observer_v", observer_version_);
    def("observer_en", observer_enabled);
    def("observer_peer_on", observer_peer_present);
    def("observer_peer_type", observer_peer_type);
    def("observer_peer_addr", observer_peer_address, sizeof(observer_peer_address));
  }

 public:
  template <typename Save>
  bool persistEnabled(bool enabled, Save save) {
    const uint8_t previous_enabled = observer_enabled;
    const uint8_t previous_version = observer_version_;
    const uint8_t previous_peer_present = observer_peer_present;
    const uint8_t previous_peer_type = observer_peer_type;
    uint8_t previous_peer_address[sizeof(observer_peer_address)];
    memcpy(previous_peer_address, observer_peer_address, sizeof(previous_peer_address));

    observer_version_ = kObserverVersion;
    observer_enabled = enabled ? 1 : 0;
    if (previous_version != kObserverVersion) {
      observer_peer_present = 0;
      observer_peer_type = 0;
      memset(observer_peer_address, 0, sizeof(observer_peer_address));
    }
    if (save()) return true;
    observer_enabled = previous_enabled;
    observer_version_ = previous_version;
    observer_peer_present = previous_peer_present;
    observer_peer_type = previous_peer_type;
    memcpy(observer_peer_address, previous_peer_address, sizeof(previous_peer_address));
    return false;
  }

  template <typename Save>
  bool persistPeer(bool has_peer, const PeerIdentity& peer, Save save) {
    const uint8_t previous_present = observer_peer_present;
    const uint8_t previous_version = observer_version_;
    const uint8_t previous_enabled = observer_enabled;
    const uint8_t previous_type = observer_peer_type;
    uint8_t previous_address[sizeof(observer_peer_address)];
    memcpy(previous_address, observer_peer_address, sizeof(previous_address));

    observer_version_ = kObserverVersion;
    observer_peer_present = has_peer ? 1 : 0;
    observer_peer_type = peer.type;
    memcpy(observer_peer_address, peer.address, sizeof(peer.address));
    if (save()) return true;
    observer_peer_present = previous_present;
    observer_version_ = previous_version;
    observer_enabled = previous_enabled;
    observer_peer_type = previous_type;
    memcpy(observer_peer_address, previous_address, sizeof(previous_address));
    return false;
  }
};

}  // namespace observer
