#pragma once

#include <helpers/CommonCLI.h>
#include "ObserverPolicy.h"

namespace observer {

// ConfigSerializer keeps at most CONFIG_MAX_KEYLEN-1 characters of a key and a longer key
// in prefs.json aborts the whole load, so every key below must stay within that limit.
namespace prefs_key {
constexpr size_t len(const char* s) { return *s ? 1 + len(s + 1) : 0; }
constexpr const char* kVersion = "observer_v";
constexpr const char* kEnabled = "observer_en";
constexpr const char* kPeerPresent = "observer_p_on";
constexpr const char* kPeerType = "observer_p_type";
constexpr const char* kPeerAddress = "observer_p_addr";
static_assert(len(kVersion) < CONFIG_MAX_KEYLEN, "prefs key too long");
static_assert(len(kEnabled) < CONFIG_MAX_KEYLEN, "prefs key too long");
static_assert(len(kPeerPresent) < CONFIG_MAX_KEYLEN, "prefs key too long");
static_assert(len(kPeerType) < CONFIG_MAX_KEYLEN, "prefs key too long");
static_assert(len(kPeerAddress) < CONFIG_MAX_KEYLEN, "prefs key too long");
// Upstream's default CONFIG_MAX_KEYLEN, independent of any build override.
constexpr size_t kDefaultMaxKeyLen = 16;
static_assert(len(kVersion) < kDefaultMaxKeyLen, "prefs key too long for the default limit");
static_assert(len(kEnabled) < kDefaultMaxKeyLen, "prefs key too long for the default limit");
static_assert(len(kPeerPresent) < kDefaultMaxKeyLen, "prefs key too long for the default limit");
static_assert(len(kPeerType) < kDefaultMaxKeyLen, "prefs key too long for the default limit");
static_assert(len(kPeerAddress) < kDefaultMaxKeyLen, "prefs key too long for the default limit");
#ifdef OBSERVER_LEGACY_PREF_KEYS
// Names written by firmware older than the rename. Read-only, see structure().
constexpr const char* kLegacyPeerPresent = "observer_peer_on";
constexpr const char* kLegacyPeerType = "observer_peer_type";
constexpr const char* kLegacyPeerAddress = "observer_peer_addr";
static_assert(len(kLegacyPeerPresent) < CONFIG_MAX_KEYLEN, "build CONFIG_MAX_KEYLEN too small for legacy keys");
static_assert(len(kLegacyPeerType) < CONFIG_MAX_KEYLEN, "build CONFIG_MAX_KEYLEN too small for legacy keys");
static_assert(len(kLegacyPeerAddress) < CONFIG_MAX_KEYLEN, "build CONFIG_MAX_KEYLEN too small for legacy keys");
#endif
}  // namespace prefs_key

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
    def(prefs_key::kVersion, observer_version_);
    def(prefs_key::kEnabled, observer_enabled);
#ifdef OBSERVER_LEGACY_PREF_KEYS
    // A new name always wins; an old name is used only while its new name has not been read
    // (file order is not relied on). Old names are never written.
    noteNewKey(prefs_key::kPeerPresent, kSeenPresent);
    noteNewKey(prefs_key::kPeerType, kSeenType);
    noteNewKey(prefs_key::kPeerAddress, kSeenAddress);
    if (useLegacyKey(prefs_key::kLegacyPeerPresent, kSeenPresent)) def(prefs_key::kLegacyPeerPresent, observer_peer_present);
    if (useLegacyKey(prefs_key::kLegacyPeerType, kSeenType)) def(prefs_key::kLegacyPeerType, observer_peer_type);
    if (useLegacyKey(prefs_key::kLegacyPeerAddress, kSeenAddress)) {
      def(prefs_key::kLegacyPeerAddress, observer_peer_address, sizeof(observer_peer_address));
    }
#endif
    def(prefs_key::kPeerPresent, observer_peer_present);
    def(prefs_key::kPeerType, observer_peer_type);
    def(prefs_key::kPeerAddress, observer_peer_address, sizeof(observer_peer_address));
  }

#ifdef OBSERVER_LEGACY_PREF_KEYS
  static constexpr uint8_t kSeenPresent = 1, kSeenType = 2, kSeenAddress = 4;
  uint8_t seen_new_keys_ = 0;
  bool legacy_keys_used_ = false;

  bool reading() const { return _context && _context->op() == READ; }
  void noteNewKey(const char* key, uint8_t bit) {
    if (reading() && _context->keyMatch(getDepth(), key)) seen_new_keys_ |= bit;
  }
  bool useLegacyKey(const char* key, uint8_t bit) {
    if (!reading() || (seen_new_keys_ & bit) || !_context->keyMatch(getDepth(), key)) return false;
    legacy_keys_used_ = true;
    return true;
  }
#endif

 public:
#ifdef OBSERVER_LEGACY_PREF_KEYS
  // True when the last load took a value from an old-name key, so the file still holds old names.
  bool legacyKeysUsed() const { return legacy_keys_used_; }
  void clearLegacyKeysUsed() { legacy_keys_used_ = false; }
#endif

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
