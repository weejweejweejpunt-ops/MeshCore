#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ObserverPolicy.h"

namespace observer {

struct PreferenceStore {
  virtual ~PreferenceStore() = default;
  virtual bool read(const char* key, uint8_t* output, size_t length) = 0;
  virtual bool write(const char* key, const uint8_t* value, size_t length) = 0;
  virtual bool remove(const char* key) = 0;
};

class Preferences {
 public:
  explicit Preferences(PreferenceStore& store) : store_(store) {}

  bool load(bool& enabled, PeerIdentity& peer, bool& has_peer) {
    enabled = false;
    has_peer = false;
    uint8_t version = 0;
    if (!store_.read("observer_v", &version, 1)) return true;
    if (version != kVersion) return false;
    uint8_t enabled_byte = 0;
    if (store_.read("observer_en", &enabled_byte, 1)) enabled = enabled_byte == 1;
    uint8_t identity[7]{};
    if (store_.read("observer_peer", identity, sizeof(identity))) {
      peer.type = identity[0];
      memcpy(peer.address, identity + 1, 6);
      has_peer = true;
    }
    return true;
  }

  bool saveEnabled(bool enabled) {
    const uint8_t version = kVersion;
    const uint8_t value = enabled ? 1 : 0;
    return store_.write("observer_v", &version, 1) && store_.write("observer_en", &value, 1);
  }

  bool savePeer(bool has_peer, const PeerIdentity& peer) {
    const uint8_t version = kVersion;
    if (!store_.write("observer_v", &version, 1)) return false;
    if (!has_peer) return store_.remove("observer_peer");
    uint8_t identity[7]{};
    identity[0] = peer.type;
    memcpy(identity + 1, peer.address, 6);
    return store_.write("observer_peer", identity, sizeof(identity));
  }

  static constexpr uint8_t kVersion = 1;

 private:
  PreferenceStore& store_;
};

}  // namespace observer
